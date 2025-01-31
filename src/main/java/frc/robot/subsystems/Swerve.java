package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;
  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  private TimeInterpolatableBuffer<Rotation2d> rotationBuffer =
      TimeInterpolatableBuffer.createBuffer(1.5);

  private Field2d field = new Field2d();

  Transform2d aprilTagOffset;

  public static record PoseEstimate(Pose3d estimatedPose, double timestamp, Vector<N3> standardDevs)
      implements Comparable<PoseEstimate> {
    @Override
    public int compareTo(PoseEstimate other) {
      if (timestamp > other.timestamp) {
        return 1;
      } else if (timestamp < other.timestamp) {
        return -1;
      }
      return 0;
    }
  }

  private PhotonCamera arducamLeft = new PhotonCamera(VisionConstants.arducamLeftName);
  private PhotonPoseEstimator arducamLeftPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.aprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.arducamLeftTransform);

  private PhotonCamera arducamRight = new PhotonCamera(VisionConstants.arducamRightName);
  private PhotonPoseEstimator arducamRightPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.aprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.arducamRightTransform);

  private PhotonCamera limelight = new PhotonCamera(VisionConstants.limelightName);
  private PhotonPoseEstimator limelightPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.aprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.limelightTransform);

  private List<PhotonPipelineResult> latestarducamLeftResult;
  private List<PhotonPipelineResult> latestarducamRightResult;
  public List<PhotonPipelineResult> latestLimelightResult;

  public Transform2d bestAprilTagTransform;

  // Temporary fix for inaccurate poses while auto shooting

  private PhotonCameraSim arducamSimLeft;
  private PhotonCameraSim arducamSimTwo;
  private PhotonCameraSim limelightSim;

  private VisionSystemSim visionSim;

  public List<Pose3d> detectedTargets = new ArrayList<>();
  public List<Integer> detectedAprilTags = new ArrayList<>();
  private List<Pose3d> rejectedPoses = new ArrayList<>();
  private List<PoseEstimate> poseEstimates = new ArrayList<>();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  // private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);

    if (Utils.isSimulation()) {
      startSimThread();
    }

    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);

    // pigeon =
    //     new Pigeon2(GyroConstants.pigeonID, "Cannie"); // Need to change the the pigeon ID for
    // later

    if (Utils.isSimulation()) {
      startSimThread();
      visionSim = new VisionSystemSim("main");

      visionSim.addAprilTags(FieldConstants.aprilTagLayout);

      SimCameraProperties arducamProperties = new SimCameraProperties();
      arducamProperties.setCalibration(800, 600, Rotation2d.fromDegrees(85.4));
      arducamProperties.setCalibError(0.21, 0.10);
      arducamProperties.setFPS(28);
      arducamProperties.setAvgLatencyMs(36);
      arducamProperties.setLatencyStdDevMs(15);
      arducamProperties.setExposureTimeMs(45);

      arducamSimLeft = new PhotonCameraSim(arducamLeft, arducamProperties);
      arducamSimTwo = new PhotonCameraSim(arducamRight, arducamProperties);
      visionSim.addCamera(arducamSimLeft, VisionConstants.arducamLeftTransform);
      visionSim.addCamera(arducamSimTwo, VisionConstants.arducamRightTransform);

      arducamSimLeft.enableRawStream(true);
      arducamSimLeft.enableProcessedStream(true);
      arducamSimTwo.enableRawStream(true);
      arducamSimTwo.enableProcessedStream(true);

      SimCameraProperties limelightProperties = new SimCameraProperties();
      limelightProperties.setCalibration(960, 720, Rotation2d.fromDegrees(97.60));
      limelightProperties.setCalibError(0.21, 0.10);
      limelightProperties.setFPS(30);
      limelightProperties.setAvgLatencyMs(36);
      limelightProperties.setLatencyStdDevMs(15);
      limelightProperties.setExposureTimeMs(45);

      limelightSim = new PhotonCameraSim(limelight, limelightProperties);
      visionSim.addCamera(limelightSim, VisionConstants.limelightTransform);

      limelightSim.enableRawStream(true);
      limelightSim.enableProcessedStream(true);
      limelightSim.enableDrawWireframe(true);
    }

    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);

    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  public void configureAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose,
          this::resetPose,
          () -> getState().Speeds,
          (speeds, feedforwards) ->
              setControl(
                  pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(AutoConstants.translationPID, AutoConstants.rotationPID),
          config,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this);
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
    PathPlannerLogging.setLogActivePathCallback(
        poses -> {
          field.getObject("Trajectory").setPoses(poses);

          if (poses.isEmpty()) {
            field.getObject("Target Pose").setPoses();
          }
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> field.getObject("Target Pose").setPose(pose));

    SmartDashboard.putData("Swerve/Field", field);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Optional<Rotation2d> getRotationAtTime(double time) {
    Optional<Rotation2d> rotationAtTime = rotationBuffer.getSample(time);
    return rotationAtTime;
  }

  public Pose3d getarducamLeftPose() {
    return new Pose3d(getState().Pose).plus(VisionConstants.arducamLeftTransform);
  }

  public Pose3d getarducamRightPose() {
    return new Pose3d(getState().Pose).plus(VisionConstants.arducamRightTransform);
  }

  public Pose3d getLimelightPose() {
    return new Pose3d(getState().Pose).plus(VisionConstants.limelightTransform);
  }

  // public static Transform2d getBestTransform(List<Transform2d> transforms) {
  //   // Define a comparator to sort transforms by their magnitude
  //   Comparator<Transform2d> comparator =
  //       Comparator.comparingDouble(
  //           transform ->
  //               transform.getTranslation().getNorm()
  //                   + Math.abs(transform.getRotation().getRadians()));

  //   // Sort the list of transforms in ascending order of magnitude
  //   Collections.sort(transforms, comparator);

  //   // Return the first transform (which has the smallest magnitude)
  //   return transforms.get(0);
  // }

  // public Transform2d getBestAprilTag(List<PhotonPipelineResult> latestResult) {

  //   List<Transform2d> targets = new ArrayList<>();
  //   for (PhotonPipelineResult result : latestResult) {

  //     Optional<EstimatedRobotPose> optionalVisionPose = limelightPoseEstimator.update(result);
  //     EstimatedRobotPose visionPose = optionalVisionPose.get();

  //     for (PhotonTrackedTarget target : visionPose.targetsUsed) {
  //       targets.add(
  //           new Transform2d(
  //               target.getBestCameraToTarget().getX(),
  //               target.getBestCameraToTarget().getY(),
  //               target.getBestCameraToTarget().getRotation().toRotation2d()));
  //     }
  //   }
  //   Transform2d bestTransform = getBestTransform(targets);
  //   return bestTransform;
  // }

  private Vector<N3> getVisionStdDevs(
      int tagCount, double averageDistance, double baseStandardDev) {
    double stdDevScale = 1 + (averageDistance * averageDistance) / 30;

    return VecBuilder.fill(
        baseStandardDev * stdDevScale, baseStandardDev * stdDevScale, Double.POSITIVE_INFINITY);
  }

  private boolean isValidPose(
      Pose3d visionPose, double averageDistance, int detectedTargets, double timestampSeconds) {
    if (averageDistance > 4.5) { // 6.5
      return false;
    }

    if (DriverStation.isAutonomous()) {
      return false;
    }

    if (averageDistance > 3 && detectedTargets < 2) {//4
      return false;
    }

    // If it thinks the robot is out of field bounds
    if (visionPose.getX() < 0.0
        || visionPose.getX() > FieldConstants.aprilTagLayout.getFieldLength()
        || visionPose.getY() < 0.0
        || visionPose.getY() > FieldConstants.aprilTagLayout.getFieldWidth()
        || visionPose.getZ()
            < -0.5 // To account for minor inaccuracies in the camera location on the robot
        || visionPose.getZ() > 1.6) {
      return false;
    }

    Optional<Rotation2d> angleAtTime = getRotationAtTime(timestampSeconds);
    if (angleAtTime.isEmpty()) {
      angleAtTime = Optional.of(getState().Pose.getRotation());
    }

    Rotation2d angleDifference = angleAtTime.get().minus(visionPose.getRotation().toRotation2d());

    double angleTolerance =
        DriverStation.isAutonomous() ? 8.0 : (detectedTargets >= 2) ? 25.0 : 15.0;

    if (Math.abs(angleDifference.getDegrees()) > angleTolerance) {
      return false;
    }

    return true;
  }

  private void updateVisionPoses(
      List<PhotonPipelineResult> latestResults,
      PhotonPoseEstimator poseEstimator,
      Transform3d cameraTransform,
      double tagStdDev) {
    if (latestResults.isEmpty()) {
      return;
    }

    poseEstimator.setReferencePose(getState().Pose);

    for (PhotonPipelineResult result : latestResults) {
      Optional<EstimatedRobotPose> optionalVisionPose = poseEstimator.update(result);
      if (optionalVisionPose.isEmpty()) {
        continue;
      }

      EstimatedRobotPose visionPose = optionalVisionPose.get();

      double totalDistance = 0.0;
      int tagCount = 0;

      for (PhotonTrackedTarget target : visionPose.targetsUsed) {
        tagCount++;
        totalDistance +=
            target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
      }

      double averageDistance = totalDistance / tagCount;

      if (!isValidPose(
          visionPose.estimatedPose,
          averageDistance,
          visionPose.targetsUsed.size(),
          visionPose.timestampSeconds)) {
        rejectedPoses.add(visionPose.estimatedPose);
        return;
      }

      poseEstimates.add(
          new PoseEstimate(
              visionPose.estimatedPose,
              visionPose.timestampSeconds,
              getVisionStdDevs(tagCount, averageDistance, tagStdDev)));

      for (PhotonTrackedTarget target : visionPose.targetsUsed) {
        int aprilTagID = target.getFiducialId();

        Optional<Pose3d> tagPose = FieldConstants.aprilTagLayout.getTagPose(aprilTagID);
        if (tagPose.isEmpty()) {
          continue;
        }

        detectedAprilTags.add(aprilTagID);
        detectedTargets.add(tagPose.get());
      }
    }
  }

  public void updateVisionPoseEstimates() {
    poseEstimates.clear();
    detectedTargets.clear();
    rejectedPoses.clear();

    updateVisionPoses(
        latestarducamLeftResult,
        arducamLeftPoseEstimator,
        VisionConstants.arducamLeftTransform,
        Units.inchesToMeters(2.5));
    updateVisionPoses(
        latestarducamRightResult,
        arducamRightPoseEstimator,
        VisionConstants.arducamRightTransform,
        Units.inchesToMeters(2.5));
    updateVisionPoses(
        latestLimelightResult,
        limelightPoseEstimator,
        VisionConstants.limelightTransform,
        Units.inchesToMeters(2.5));

    Collections.sort(poseEstimates);

    for (PoseEstimate poseEstimate : poseEstimates) {
      addVisionMeasurement(
          poseEstimate.estimatedPose().toPose2d(),
          poseEstimate.timestamp(),
          poseEstimate.standardDevs());
    }

    field
        .getObject("Detected Targets")
        .setPoses(detectedTargets.stream().map(p -> p.toPose2d()).toArray(Pose2d[]::new));
    field
        .getObject("Rejected Poses")
        .setPoses(rejectedPoses.stream().map(p -> p.toPose2d()).toArray(Pose2d[]::new));
  }

  public List<PhotonPipelineResult> getLimelightResults() {
    return latestLimelightResult;
  }

  public List<PhotonPipelineResult> getarducamLeftResults() {
    return latestarducamLeftResult;
  }

  public List<PhotonPipelineResult> getarducamRightResults() {
    return latestarducamRightResult;
  }

  // public Command CoralAlign(String angleOffset) {

  //   if (angleOffset == "Left") {
  //     aprilTagOffset = new Transform2d(0, VisionConstants.aprilTagReefOffset, new Rotation2d(0));
  //   }
  //   if (angleOffset == "Right") {
  //     aprilTagOffset = new Transform2d(0, -VisionConstants.aprilTagReefOffset, new
  // Rotation2d(0));
  //   }

  //   Transform2d transform = getBestAprilTag(getLimelightResults());

  //   transform = transform.plus(aprilTagOffset);

  //   Pose2d aprilTagPose = getState().Pose.transformBy(transform);

  //   PathConstraints constraints =
  //       new PathConstraints(12.0, 10.0, Units.degreesToRadians(720),
  // Units.degreesToRadians(720));

  //   Command pathfind = AutoBuilder.pathfindToPose(aprilTagPose, constraints, 0.0);

  //   return pathfind;
  // }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.quasistatic(direction);
  }

  public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.dynamic(direction);
  }

  public Command sysIdQuasistaticTranslation(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineTranslation.quasistatic(direction);
  }

  public Command sysIdDynamicTranslation(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineTranslation.dynamic(direction);
  }

  public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineSteer.quasistatic(direction);
  }

  public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineSteer.dynamic(direction);
  }

  @Override
  public void periodic() {
    Pose2d robotPose = getState().Pose;

    field.setRobotPose(robotPose);

    latestarducamLeftResult = arducamLeft.getAllUnreadResults();
    latestarducamRightResult = arducamRight.getAllUnreadResults();
    latestLimelightResult = limelight.getAllUnreadResults();

    updateVisionPoseEstimates();

    final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    final NetworkTable swerveStateTable = inst.getTable("DriveState");
    final StructPublisher<Pose2d> drivePose =
        swerveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    final StructPublisher<ChassisSpeeds> driveSpeeds =
        swerveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    final StructArrayPublisher<SwerveModuleState> driveModuleStates =
        swerveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
        swerveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
        swerveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
            .publish();

    drivePose.set(getState().Pose);
    driveSpeeds.set(getState().Speeds);
    driveModuleStates.set(getState().ModuleStates);
    driveModuleTargets.set(getState().ModuleTargets);
    driveModulePositions.set(getState().ModulePositions);

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
  }

  public void method() {}

  @Override
  public void simulationPeriodic() {
    // Update camera simulation
    Pose2d robotPose = getState().Pose;

    field.getObject("EstimatedRobot").setPose(robotPose);

    visionSim.update(robotPose);

    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
