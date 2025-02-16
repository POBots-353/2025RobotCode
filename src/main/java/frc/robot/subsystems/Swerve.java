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
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefDefinitePoses;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.AllianceUtil;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
@Logged(strategy = Strategy.OPT_IN)
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

  private Field2d field = new Field2d();

  private int bestTargetID;
  private Pose2d leftPose;
  private Pose2d rightPose;

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

  private List<PhotonPipelineResult> latestArducamLeftResult;
  private List<PhotonPipelineResult> latestArducamRightResult;
  private List<PhotonPipelineResult> latestLimelightResult;

  private PhotonCameraSim arducamSimLeft;
  private PhotonCameraSim arducamSimTwo;
  private PhotonCameraSim limelightSim;

  private VisionSystemSim visionSim;

  @Logged(name = "Detected Targets")
  private List<Pose3d> detectedTargets = new ArrayList<>();

  private List<Integer> detectedAprilTags = new ArrayList<>();

  @Logged(name = "Rejected Poses")
  private List<Pose3d> rejectedPoses = new ArrayList<>();

  private List<PoseEstimate> poseEstimates = new ArrayList<>();

  // Never called, only used to allow logging the poses being used
  @Logged(name = "Accepted Poses")
  public List<Pose3d> acceptedPosesList() {
    return poseEstimates.stream().map((p) -> p.estimatedPose()).toList();
  }

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
      initVisionSim();
    }
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

    if (Utils.isSimulation()) {
      startSimThread();
      initVisionSim();
    }
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
      initVisionSim();
    }
  }

  private void initVisionSim() {
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
    limelightProperties.setCalibration(640, 480, Rotation2d.fromDegrees(60)); // 960 720 97
    limelightProperties.setCalibError(0.58, 0.10);
    limelightProperties.setFPS(26); // 30
    limelightProperties.setAvgLatencyMs(70); // 36
    limelightProperties.setLatencyStdDevMs(15);
    limelightProperties.setExposureTimeMs(45);

    limelightSim = new PhotonCameraSim(limelight, limelightProperties);
    visionSim.addCamera(limelightSim, VisionConstants.limelightTransform);

    limelightSim.enableRawStream(true);
    limelightSim.enableProcessedStream(true);
    limelightSim.enableDrawWireframe(true);
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
            setControl(
                pathApplyRobotSpeeds
                    .withSpeeds(new ChassisSpeeds())
                    .withWheelForceFeedforwardsX(new double[] {})
                    .withWheelForceFeedforwardsY(new double[] {}));
          }
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> field.getObject("Target Pose").setPose(pose));

    SmartDashboard.putData("Swerve/Field", field);
  }

  public void updateBestAlignmentPose() {
    List<PhotonPipelineResult> latestResult = latestLimelightResult;

    if (latestResult == null || latestResult.isEmpty()) {
      return;
    }

    List<PhotonTrackedTarget> validTargets = new ArrayList<>();
    for (PhotonPipelineResult result : latestResult) {
      if (!result.hasTargets()) continue;

      for (PhotonTrackedTarget target : result.getTargets()) {
        if (AllianceUtil.getReefIds().contains(target.getFiducialId())) {
          validTargets.add(target);
        }
      }
    }

    if (validTargets.isEmpty()) {
      return;
    }

    validTargets.sort(
        Comparator.comparingDouble(
            target -> target.getBestCameraToTarget().getTranslation().getNorm()));

    PhotonTrackedTarget bestTarget = validTargets.get(0);

    bestTargetID = bestTarget.getFiducialId();
    Double desiredRotation = FieldConstants.aprilTagAngles.getOrDefault(bestTargetID, 0.0);
    Transform2d bestTransform =
        new Transform2d(
            bestTarget.getBestCameraToTarget().getX(),
            bestTarget.getBestCameraToTarget().getY(),
            bestTarget.getBestCameraToTarget().getRotation().toRotation2d());

    Transform2d leftAprilTagOffset =
        new Transform2d(
            -SwerveConstants.centerToBumber,
            FieldConstants.left_aprilTagOffsets.getOrDefault(bestTargetID, 0.0),
            new Rotation2d(0));
    Transform2d rightAprilTagOffset =
        new Transform2d(
            -SwerveConstants.centerToBumber,
            FieldConstants.right_aprilTagOffsets.getOrDefault(bestTargetID, 0.0),
            new Rotation2d(0));

    leftPose =
        new Pose2d(
            getState()
                .Pose
                .transformBy(VisionConstants.limelightTransform2d)
                .transformBy(
                    new Transform2d(
                        bestTransform.getTranslation().plus(leftAprilTagOffset.getTranslation()),
                        Rotation2d.fromDegrees(0)))
                .getTranslation(),
            Rotation2d.fromDegrees(desiredRotation));
    rightPose =
        new Pose2d(
            getState()
                .Pose
                .transformBy(VisionConstants.limelightTransform2d)
                .transformBy(
                    new Transform2d(
                        bestTransform.getTranslation().plus(rightAprilTagOffset.getTranslation()),
                        Rotation2d.fromDegrees(0)))
                .getTranslation(),
            Rotation2d.fromDegrees(desiredRotation));

    SmartDashboard.putNumber("Swerve/Goal Rotation", desiredRotation);
    SmartDashboard.putNumber("Swerve/Best Tag ID", bestTargetID);
    SmartDashboard.putNumber("Swerve/Current Rotation", getState().Pose.getRotation().getDegrees());

    SmartDashboard.putNumber("Swerve/Right X Pose", rightPose.getX());
    SmartDashboard.putNumber("Swerve/Right Y Pose", rightPose.getY());
    SmartDashboard.putNumber("Swerve/Left X Pose", leftPose.getX());
    SmartDashboard.putNumber("Swerve/Left Y Pose", leftPose.getY());
  }

  public Command reefAlign(boolean leftAlign) {
    return new DeferredCommand(
        () -> {
          Pose2d goalPose = leftAlign ? leftPose : rightPose;
          SmartDashboard.putNumber("Swerve/Attempted Pose X", goalPose.getX());
          SmartDashboard.putNumber("Swerve/Attempted Pose Y", goalPose.getY());
          // return new InstantCommand();
          return AutoBuilder.pathfindToPose(goalPose, SwerveConstants.pathConstraints, 0.0);
        },
        Set.of(this));
  }

  public Command reefAlignNoVision(boolean leftAlign) {
    return new DeferredCommand(
        () -> {
          Pose2d robotPose = getState().Pose;
          Pose2d nearestPose = Pose2d.kZero;
          if (AllianceUtil.isRedAlliance()) {
            if (leftAlign) {
              nearestPose = robotPose.nearest(ReefDefinitePoses.redReefDefiniteLeftPoses);
            } else {
              nearestPose = robotPose.nearest(ReefDefinitePoses.redReefDefiniteRightPoses);
            }
          } else {
            if (leftAlign) {
              nearestPose = robotPose.nearest(ReefDefinitePoses.blueReefDefiniteLeftPoses);
            } else {
              nearestPose = robotPose.nearest(ReefDefinitePoses.blueReefDefiniteRightPoses);
            }
          }
          return AutoBuilder.pathfindToPose(nearestPose, SwerveConstants.pathConstraints, 0.0);
        },
        Set.of(this));
  }

  public PathPlannerPath getNearestPickupPath() {
    Pose2d closestStation;
    PathPlannerPath path = null;

    List<Pose2d> redStations =
        List.of(FieldConstants.redStationLeft, FieldConstants.redStationRight);
    List<Pose2d> blueStations =
        List.of(FieldConstants.blueStationLeft, FieldConstants.blueStationRight);

    try {
      if (AllianceUtil.isRedAlliance()) {
        closestStation = getState().Pose.nearest(redStations);
        if (redStations.indexOf(closestStation) == 0) {
          path = PathPlannerPath.fromPathFile("Human Player Pickup Left");
        } else {
          path = PathPlannerPath.fromPathFile("Human Player Pickup Right");
        }
      } else {
        closestStation = getState().Pose.nearest(blueStations);
        if (blueStations.indexOf(closestStation) == 0) {
          path = PathPlannerPath.fromPathFile("Human Player Pickup Left");
        } else {
          path = PathPlannerPath.fromPathFile("Human Player Pickup Right");
        }
      }
    } catch (IOException | FileVersionException | ParseException e) {
      System.err.println("Error loading PathPlanner path: " + e.getMessage());
      e.printStackTrace();
      path = null;
    }

    return path;
  }

  public Command humanPlayerAlign() {
    return new DeferredCommand(
        () -> {
          PathPlannerPath goalPath = getNearestPickupPath();
          if (goalPath != null) {
            return AutoBuilder.pathfindThenFollowPath(goalPath, SwerveConstants.pathConstraints);
          } else {
            System.err.println("Invalid goalPath, path cannot be followed.");
            return new InstantCommand();
          }
        },
        Set.of(this));
  }

  public Command pathFindToSetup() {
    return new DeferredCommand(
        () -> {
          Pose2d closestPose;

          List<Pose2d> redSetupPoses = FieldConstants.redSetupPoses;
          List<Pose2d> blueSetupPoses = FieldConstants.blueSetupPoses;

          if (AllianceUtil.isRedAlliance()) {
            closestPose = getState().Pose.nearest(redSetupPoses);

          } else {
            closestPose = getState().Pose.nearest(blueSetupPoses);
          }

          Pose2d goalSetUpPose = closestPose;
          return AutoBuilder.pathfindToPose(goalSetUpPose, SwerveConstants.pathConstraints);
        },
        Set.of(this));
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

  public Pose3d getArducamLeftPose() {
    return new Pose3d(getState().Pose).plus(VisionConstants.arducamLeftTransform);
  }

  public Pose3d getArducamRightPose() {
    return new Pose3d(getState().Pose).plus(VisionConstants.arducamRightTransform);
  }

  public Pose3d getLimelightPose() {
    return new Pose3d(getState().Pose).plus(VisionConstants.limelightTransform);
  }

  private Pose3d calculateSingleTagPose(
      PhotonTrackedTarget target,
      Pose3d tagPoseOnField,
      Pose2d poseAtTime,
      Transform3d cameraTransform) {
    Rotation2d yaw = Rotation2d.fromDegrees(-target.getYaw());
    Rotation2d pitch = Rotation2d.fromDegrees(target.getPitch());

    Transform3d cameraToRobot3d =
        new Transform3d(cameraTransform.getTranslation(), cameraTransform.getRotation()).inverse();

    double distanceMagnitude =
        target.getBestCameraToTarget().getTranslation().getNorm() * pitch.getCos();

    Translation3d cameraToTargetTranslation =
        new Translation3d(
            yaw.getCos() * distanceMagnitude,
            yaw.getSin() * distanceMagnitude,
            pitch.getSin() * distanceMagnitude);
    Rotation3d cameraToTagRotation =
        tagPoseOnField
            .getRotation()
            .minus(new Rotation3d(poseAtTime.getRotation()))
            .plus(cameraToRobot3d.getRotation());

    return PhotonUtils.estimateFieldToRobotAprilTag(
        new Transform3d(cameraToTargetTranslation, cameraToTagRotation),
        tagPoseOnField,
        cameraToRobot3d);
  }

  private Vector<N3> getVisionStdDevs(
      int tagCount, double averageDistance, double baseStandardDev) {
    double stdDevScale = 1 + (averageDistance * averageDistance) / 30;

    return VecBuilder.fill(
        baseStandardDev * stdDevScale, baseStandardDev * stdDevScale, Double.POSITIVE_INFINITY);
  }

  private boolean isOutOfBounds(Pose3d visionPose) {
    // Allow the robot to be just slightly off the field
    final double fieldTolerance = Units.inchesToMeters(2.5);

    return visionPose.getX() < -fieldTolerance
        || visionPose.getX() > FieldConstants.aprilTagLayout.getFieldLength() + fieldTolerance
        || visionPose.getY() < -fieldTolerance
        || visionPose.getY() > FieldConstants.aprilTagLayout.getFieldWidth() + fieldTolerance
        || visionPose.getZ() < -0.5
        || visionPose.getZ() > 1.6;
  }

  private boolean isValidSingleTagPose(Pose3d visionPose, double distance) {
    if (distance > 4.5) {
      return false;
    }

    if (DriverStation.isAutonomous()) {
      return false;
    }

    if (isOutOfBounds(visionPose)) {
      return false;
    }

    return true;
  }

  private boolean isValidMultitagPose(
      Pose3d visionPose, double averageDistance, int detectedTargets, double timestampSeconds) {
    if (averageDistance > 4.5) { // 6.5
      return false;
    }

    if (DriverStation.isAutonomous()) {
      return false;
    }

    if (isOutOfBounds(visionPose)) {
      return false;
    }

    Optional<Rotation2d> rotationAtTime =
        samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds)).map((pose) -> pose.getRotation());

    if (rotationAtTime.isEmpty()) {
      return false;
    }

    Rotation2d angleDifference =
        rotationAtTime.get().minus(visionPose.getRotation().toRotation2d());

    double angleTolerance = DriverStation.isAutonomous() ? 8.0 : 15.0;

    if (Math.abs(angleDifference.getDegrees()) > angleTolerance) {
      return false;
    }

    return true;
  }

  private void updateVisionPoses(
      List<PhotonPipelineResult> latestResults,
      PhotonPoseEstimator poseEstimator,
      Transform3d cameraTransform,
      double baseSingleTagStdDev,
      double baseMultiTagStdDev) {
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

      if (tagCount > 1
          && !isValidMultitagPose(
              visionPose.estimatedPose,
              averageDistance,
              visionPose.targetsUsed.size(),
              visionPose.timestampSeconds)) {
        rejectedPoses.add(visionPose.estimatedPose);
        return;
      }

      if (tagCount > 1) {
        poseEstimates.add(
            new PoseEstimate(
                visionPose.estimatedPose,
                visionPose.timestampSeconds,
                getVisionStdDevs(tagCount, averageDistance, baseMultiTagStdDev)));
      } else {
        PhotonTrackedTarget target = visionPose.targetsUsed.get(0);
        Optional<Pose2d> robotPoseAtTime =
            samplePoseAt(Utils.fpgaToCurrentTime(visionPose.timestampSeconds));
        Optional<Pose3d> tagOnField =
            FieldConstants.aprilTagLayout.getTagPose(target.getFiducialId());

        if (robotPoseAtTime.isEmpty() || tagOnField.isEmpty()) {
          continue;
        }

        Pose3d singleTagPose =
            calculateSingleTagPose(
                target, tagOnField.get(), robotPoseAtTime.get(), cameraTransform);

        if (!isValidSingleTagPose(singleTagPose, averageDistance)) {
          rejectedPoses.add(singleTagPose);
          continue;
        }

        poseEstimates.add(
            new PoseEstimate(
                singleTagPose,
                visionPose.timestampSeconds,
                getVisionStdDevs(tagCount, averageDistance, baseSingleTagStdDev)));
      }

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

  private void updateVisionPoseEstimates() {
    poseEstimates.clear();
    detectedTargets.clear();
    rejectedPoses.clear();

    updateVisionPoses(
        latestArducamLeftResult,
        arducamLeftPoseEstimator,
        VisionConstants.arducamLeftTransform,
        Units.inchesToMeters(3.0),
        Units.inchesToMeters(2.5));
    updateVisionPoses(
        latestArducamRightResult,
        arducamRightPoseEstimator,
        VisionConstants.arducamRightTransform,
        Units.inchesToMeters(3.0),
        Units.inchesToMeters(2.5));
    updateVisionPoses(
        latestLimelightResult,
        limelightPoseEstimator,
        VisionConstants.limelightTransform,
        Units.inchesToMeters(3.0),
        Units.inchesToMeters(2.5));

    Collections.sort(poseEstimates);

    for (PoseEstimate poseEstimate : poseEstimates) {
      addVisionMeasurement(
          poseEstimate.estimatedPose().toPose2d(),
          Utils.fpgaToCurrentTime(poseEstimate.timestamp()),
          poseEstimate.standardDevs());
    }
  }

  public List<PhotonPipelineResult> getLimelightResults() {
    return latestLimelightResult;
  }

  public List<PhotonPipelineResult> getArducamLeftResults() {
    return latestArducamLeftResult;
  }

  public List<PhotonPipelineResult> getArducamRightResults() {
    return latestArducamRightResult;
  }

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
    double startTime = Timer.getFPGATimestamp();
    field.setRobotPose(getState().Pose);

    latestArducamLeftResult = arducamLeft.getAllUnreadResults();
    latestArducamRightResult = arducamRight.getAllUnreadResults();
    latestLimelightResult = limelight.getAllUnreadResults();

    updateVisionPoseEstimates();
    updateBestAlignmentPose();

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

    double runtime = (Timer.getFPGATimestamp() - startTime) * 1000.0;
    SmartDashboard.putNumber("Swerve/Periodic Runtime (ms)", runtime);
  }

  @Override
  public void simulationPeriodic() {
    // Update camera simulation
    Pose2d robotPose = getState().Pose;

    field.getObject("EstimatedRobot").setPose(robotPose);

    visionSim.update(robotPose);
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
