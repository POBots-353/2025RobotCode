package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefDefinitePoses;
import frc.robot.Constants.PreMatchConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.AllianceUtil;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
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

  protected List<Alert> prematchAlerts = new ArrayList<Alert>();
  protected String systemStatus = "Pre-Match not ran";

  private SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);

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

  private double preMatchTranslationalTolerance = 0.1;

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
  private Optional<Matrix<N3, N3>> arducamLeftMatrix = Optional.empty();
  private Optional<Matrix<N8, N1>> arducamLeftDistCoeffs = Optional.empty();

  private PhotonCamera arducamRight = new PhotonCamera(VisionConstants.arducamRightName);
  private PhotonPoseEstimator arducamRightPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.aprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.arducamRightTransform);
  private Optional<Matrix<N3, N3>> arducamRightMatrix = Optional.empty();
  private Optional<Matrix<N8, N1>> arducamRightDistCoeffs = Optional.empty();

  private PhotonCamera arducamFront = new PhotonCamera(VisionConstants.arducamFrontName);
  private PhotonPoseEstimator arducamFrontPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.aprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.arducamFrontTransform);
  private Optional<Matrix<N3, N3>> arducamFrontMatrix = Optional.empty();
  private Optional<Matrix<N8, N1>> arducamFrontDistCoeffs = Optional.empty();

  private static final Optional<ConstrainedSolvepnpParams> constrainedPnpParams =
      Optional.of(new ConstrainedSolvepnpParams(false, 1.5));

  private List<PhotonPipelineResult> latestArducamLeftResult;
  private List<PhotonPipelineResult> latestArducamRightResult;
  private List<PhotonPipelineResult> latestArducamFrontResult;

  private PhotonCameraSim arducamSimLeft;
  private PhotonCameraSim arducamSimRight;
  private PhotonCameraSim arducamFrontSim;

  private VisionSystemSim visionSim;

  @Logged(name = "Detected Targets")
  private List<Pose3d> detectedTargets = new ArrayList<>();

  private List<Integer> detectedAprilTags = new ArrayList<>();

  @Logged(name = "Rejected Poses")
  private List<Pose3d> rejectedPoses = new ArrayList<>();

  private List<PoseEstimate> poseEstimates = new ArrayList<>();

  private SwerveDriveState stateCache = getState();

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

    initVision();

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

    initVision();

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

    initVision();

    if (Utils.isSimulation()) {
      startSimThread();
      initVisionSim();
    }
  }

  private void initVision() {
    arducamRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    arducamLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    arducamFrontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  private void initVisionSim() {
    visionSim = new VisionSystemSim("main");

    visionSim.addAprilTags(FieldConstants.aprilTagLayout);

    Matrix<N3, N3> calibError =
        new Matrix<>(
            Nat.N3(),
            Nat.N3(),
            new double[] {
              689.4460449566128,
              0,
              441.7426355934763,
              0,
              688.5536260717645,
              292.82342214293885,
              0,
              0,
              1
            });
    Vector<N8> distCoefficients =
        VecBuilder.fill(
            0.03728225626399143,
            -0.022115127374557862,
            0.0001637647682633715,
            0.0003334141823199474,
            -0.03915318108680384,
            -0.0015121698705335032,
            0.0009546599698249316,
            0.0016260200999396255);

    SimCameraProperties arducamProperties =
        new SimCameraProperties()
            .setCalibration(800, 600, calibError, distCoefficients)
            .setCalibError(0.21, 0.10)
            .setFPS(28)
            .setAvgLatencyMs(36)
            .setLatencyStdDevMs(15)
            .setExposureTimeMs(45);

    arducamSimLeft = new PhotonCameraSim(arducamLeft, arducamProperties);
    arducamSimRight = new PhotonCameraSim(arducamRight, arducamProperties);
    arducamFrontSim = new PhotonCameraSim(arducamFront, arducamProperties);
    visionSim.addCamera(arducamSimLeft, VisionConstants.arducamLeftTransform);
    visionSim.addCamera(arducamSimRight, VisionConstants.arducamRightTransform);
    visionSim.addCamera(arducamFrontSim, VisionConstants.arducamFrontTransform);

    arducamSimLeft.enableRawStream(true);
    arducamSimLeft.enableProcessedStream(true);
    arducamSimRight.enableRawStream(true);
    arducamSimRight.enableProcessedStream(true);
    arducamFrontSim.enableRawStream(true);
    arducamFrontSim.enableProcessedStream(true);
  }

  public void configureAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> stateCache.Pose,
          this::resetPose,
          () -> stateCache.Speeds,
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
            // setControl(
            //     pathApplyRobotSpeeds
            //         .withSpeeds(new ChassisSpeeds())
            //         .withWheelForceFeedforwardsX(new double[] {})
            //         .withWheelForceFeedforwardsY(new double[] {}));
          }
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> field.getObject("Target Pose").setPose(pose));

    SmartDashboard.putData("Swerve/Field", field);
  }

  public void updateBestAlignmentPose() {
    List<PhotonPipelineResult> latestResult = latestArducamFrontResult;

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
            Rotation2d.kZero);
    Transform2d rightAprilTagOffset =
        new Transform2d(
            -SwerveConstants.centerToBumber,
            FieldConstants.right_aprilTagOffsets.getOrDefault(bestTargetID, 0.0),
            Rotation2d.kZero);

    Pose2d estimatedLeftPose =
        new Pose2d(
            stateCache
                .Pose
                .transformBy(VisionConstants.arducamFrontTransform2d)
                .transformBy(
                    new Transform2d(
                        bestTransform.getTranslation().plus(leftAprilTagOffset.getTranslation()),
                        Rotation2d.kZero))
                .getTranslation(),
            Rotation2d.fromDegrees(desiredRotation));

    Pose2d estimatedRIGHTPOSE =
        (FieldConstants.aprilTagLayout.getTagPose(bestTargetID))
            .map(Pose3d::toPose2d)
            .orElse(Pose2d.kZero)
            .transformBy(
                new Transform2d(
                        rightAprilTagOffset.getTranslation().rotateBy(Rotation2d.k180deg),
                        Rotation2d.k180deg)
                    .inverse());

    Pose2d estimatedLEFTPOSE =
        (FieldConstants.aprilTagLayout.getTagPose(bestTargetID))
            .map(Pose3d::toPose2d)
            .orElse(Pose2d.kZero)
            .transformBy(
                new Transform2d(
                        leftAprilTagOffset.getTranslation().rotateBy(Rotation2d.k180deg),
                        Rotation2d.k180deg)
                    .inverse());

    Pose2d estimatedRightPose =
        new Pose2d(
            stateCache
                .Pose
                .transformBy(VisionConstants.arducamFrontTransform2d)
                .transformBy(
                    new Transform2d(
                        bestTransform.getTranslation().plus(rightAprilTagOffset.getTranslation()),
                        Rotation2d.kZero))
                .getTranslation(),
            Rotation2d.fromDegrees(desiredRotation));

    List<Pose2d> leftPoses =
        AllianceUtil.isRedAlliance()
            ? ReefDefinitePoses.redReefDefiniteLeftPoses
            : ReefDefinitePoses.blueReefDefiniteLeftPoses;

    List<Pose2d> rightPoses =
        AllianceUtil.isRedAlliance()
            ? ReefDefinitePoses.redReefDefiniteRightPoses
            : ReefDefinitePoses.blueReefDefiniteRightPoses;

    if (isPoseWithinTolerance(estimatedLEFTPOSE, leftPoses)) {
      leftPose = estimatedLEFTPOSE;
    } else {
      return;
    }

    if (isPoseWithinTolerance(estimatedRIGHTPOSE, rightPoses)) {
      // rightPose = estimatedRightPose;
      rightPose = estimatedRIGHTPOSE;
    } else {
      return;
    }

    leftPose = estimatedLEFTPOSE;
    rightPose = estimatedRIGHTPOSE;

    SmartDashboard.putNumber("Swerve/Goal Rotation", desiredRotation);
    SmartDashboard.putNumber("Swerve/Best Tag ID", bestTargetID);
    SmartDashboard.putNumber("Swerve/Current Rotation", stateCache.Pose.getRotation().getDegrees());

    SmartDashboard.putNumber("Swerve/Right X Pose", rightPose.getX());
    SmartDashboard.putNumber("Swerve/Right Y Pose", rightPose.getY());
    SmartDashboard.putNumber("Swerve/Left X Pose", leftPose.getX());
    SmartDashboard.putNumber("Swerve/Left Y Pose", leftPose.getY());
  }

  private boolean isPoseWithinTolerance(Pose2d estimatedPose, List<Pose2d> definitePoses) {
    double tolerance = Units.inchesToMeters(3);
    Pose2d nearest = estimatedPose.nearest(definitePoses);
    return nearest.getTranslation().getDistance(estimatedPose.getTranslation()) <= tolerance;
  }

  public boolean atValidReefPose() {
    List<Pose2d> leftPoses =
        AllianceUtil.isRedAlliance()
            ? ReefDefinitePoses.redReefDefiniteLeftPoses
            : ReefDefinitePoses.blueReefDefiniteLeftPoses;

    List<Pose2d> rightPoses =
        AllianceUtil.isRedAlliance()
            ? ReefDefinitePoses.redReefDefiniteRightPoses
            : ReefDefinitePoses.blueReefDefiniteRightPoses;
    return isPoseWithinTolerance(stateCache.Pose, rightPoses)
        || isPoseWithinTolerance(stateCache.Pose, leftPoses);
  }

  public Command reefAlign(boolean leftAlign) {
    return new DeferredCommand(
            () -> {
              if (leftPose == null || rightPose == null) {
                return new InstantCommand();
              }
              Pose2d goalPose = leftAlign ? leftPose : rightPose;
              SmartDashboard.putNumber("Swerve/Attempted Pose X", goalPose.getX());
              SmartDashboard.putNumber("Swerve/Attempted Pose Y", goalPose.getY());
              // return new InstantCommand();
              return AutoBuilder.pathfindToPose(goalPose, AutoConstants.slowPathConstraints, 0.0);
            },
            Set.of(this))
        .withName("Reef Align");
  }

  public Command autoReefAlign(boolean leftAlign) {
    return new DeferredCommand(
            () -> {
              List<Pose2d> leftPoses =
                  AllianceUtil.isRedAlliance()
                      ? ReefDefinitePoses.redReefDefiniteLeftPoses
                      : ReefDefinitePoses.blueReefDefiniteLeftPoses;

              List<Pose2d> rightPoses =
                  AllianceUtil.isRedAlliance()
                      ? ReefDefinitePoses.redReefDefiniteRightPoses
                      : ReefDefinitePoses.blueReefDefiniteRightPoses;
              Pose2d autoLeftPose = stateCache.Pose.nearest(leftPoses);
              Pose2d autoRightPose = stateCache.Pose.nearest(rightPoses);

              if (autoLeftPose == null || autoRightPose == null) {
                return new InstantCommand();
              }
              Pose2d goalPose = leftAlign ? autoLeftPose : autoRightPose;
              return AutoBuilder.pathfindToPose(goalPose, AutoConstants.slowPathConstraints, 0.0);
            },
            Set.of(this))
        .withName("Auto Reef Align");
  }

  // public Command reefAlignNoPathPlanner(boolean leftAlign) {
  //   return new DeferredCommand(
  //       () -> {
  //         Pose2d robotPose = stateCache.Pose;
  //         AtomicReference<Pose2d> nearestPose = new AtomicReference<>(Pose2d.kZero);
  //         if (AllianceUtil.isRedAlliance()) {
  //           if (leftAlign) {
  //             nearestPose.set(robotPose.nearest(ReefDefinitePoses.redReefDefiniteLeftPoses));
  //           } else {
  //             nearestPose.set(robotPose.nearest(ReefDefinitePoses.redReefDefiniteRightPoses));
  //           }
  //         } else {
  //           if (leftAlign) {
  //             nearestPose.set(robotPose.nearest(ReefDefinitePoses.blueReefDefiniteLeftPoses));
  //           } else {
  //             nearestPose.set(robotPose.nearest(ReefDefinitePoses.blueReefDefiniteRightPoses));
  //           }
  //         }
  //         Supplier<Pose2d> nearestPoseSupplier = () -> nearestPose.get();
  //         Command pathCommand = new ReefAlignmentPID(nearestPoseSupplier);
  //         return pathCommand;
  //       },
  //       Set.of(this));
  // }

  public Command reefAlignNoVision(boolean leftAlign) {
    return new DeferredCommand(
            () -> {
              Pose2d robotPose = stateCache.Pose;
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
              return AutoBuilder.pathfindToPose(nearestPose, AutoConstants.midPathConstraints, 0.0);
            },
            Set.of(this))
        .withName("Reef Align No Vision");
  }

  // public PathPlannerPath getNearestPickupPath() {
  //   Pose2d closestStation;
  //   PathPlannerPath path = null;

  //   List<Pose2d> redStations =
  //       List.of(FieldConstants.redStationLeft, FieldConstants.redStationRight);
  //   List<Pose2d> blueStations =
  //       List.of(FieldConstants.blueStationLeft, FieldConstants.blueStationRight);

  //   try {
  //     if (AllianceUtil.isRedAlliance()) {
  //       closestStation = stateCache.Pose.nearest(redStations);
  //       if (redStations.indexOf(closestStation) == 0) {
  //         path = PathPlannerPath.fromPathFile("Human Player Pickup Left");
  //       } else {
  //         path = PathPlannerPath.fromPathFile("Human Player Pickup Right");
  //       }
  //     } else {
  //       closestStation = stateCache.Pose.nearest(blueStations);
  //       if (blueStations.indexOf(closestStation) == 0) {
  //         path = PathPlannerPath.fromPathFile("Human Player Pickup Left");
  //       } else {
  //         path = PathPlannerPath.fromPathFile("Human Player Pickup Right");
  //       }
  //     }
  //   } catch (IOException | FileVersionException | ParseException e) {
  //     System.err.println("Error loading PathPlanner path: " + e.getMessage());
  //     e.printStackTrace();
  //     path = null;
  //   }

  //   return path;
  // }

  // public Command humanPlayerAlign() {
  //   return new DeferredCommand(
  //       () -> {
  //         PathPlannerPath goalPath = getNearestPickupPath();
  //         if (goalPath != null) {
  //           return AutoBuilder.pathfindThenFollowPath(goalPath,
  // AutoConstants.slowPathConstraints);
  //         } else {
  //           System.err.println("Invalid goalPath, path cannot be followed.");
  //           return new InstantCommand();
  //         }
  //       },
  //       Set.of(this));
  // }

  public Command humanPlayerAlign() {
    return new DeferredCommand(
            () -> {
              Pose2d closestPose;

              List<Pose2d> redStationPoses =
                  List.of(FieldConstants.redStationLeft, FieldConstants.redStationRight);
              List<Pose2d> blueStationPoses =
                  List.of(FieldConstants.blueStationLeft, FieldConstants.blueStationRight);

              if (AllianceUtil.isRedAlliance()) {
                closestPose = stateCache.Pose.nearest(redStationPoses);

              } else {
                closestPose = stateCache.Pose.nearest(blueStationPoses);
              }

              return AutoBuilder.pathfindToPose(closestPose, AutoConstants.fastPathConstraints);
            },
            Set.of(this))
        .withName("Human Player Align");
  }

  public Command pathFindToBarge() {
    return new DeferredCommand(
            () -> {
              Pose2d closestPose =
                  AllianceUtil.isRedAlliance()
                      ? FieldConstants.redBargePose
                      : FieldConstants.blueBargePose;

              return AutoBuilder.pathfindToPose(closestPose, AutoConstants.fastPathConstraints);
            },
            Set.of(this))
        .withName("Barge Align");
  }

  public Command pathFindToProcessor() {
    return new DeferredCommand(
            () -> {
              Pose2d closestPose =
                  AllianceUtil.isRedAlliance()
                      ? FieldConstants.redProcessorPose
                      : FieldConstants.blueProcessorPose;

              return AutoBuilder.pathfindToPose(closestPose, AutoConstants.fastPathConstraints);
            },
            Set.of(this))
        .withName("Processor Align");
  }

  public Command pathFindToDirection(int direction) {
    return new DeferredCommand(
            () -> {
              List<Pose2d> setupPoses =
                  AllianceUtil.isRedAlliance()
                      ? FieldConstants.redSetupPoses
                      : FieldConstants.blueSetupPoses;

              Pose2d targetPose = setupPoses.get(direction);

              return AutoBuilder.pathfindToPose(targetPose, AutoConstants.midPathConstraints);
            },
            Set.of(this))
        .withName("Pathfind to Direction " + direction);
  }

  public Command pathFindToSetup() {
    return new DeferredCommand(
            () -> {
              Pose2d closestPose;

              List<Pose2d> redSetupPoses = FieldConstants.redSetupPoses;
              List<Pose2d> blueSetupPoses = FieldConstants.blueSetupPoses;

              if (AllianceUtil.isRedAlliance()) {
                closestPose = stateCache.Pose.nearest(redSetupPoses);

              } else {
                closestPose = stateCache.Pose.nearest(blueSetupPoses);
              }

              return AutoBuilder.pathfindToPose(closestPose, AutoConstants.midPathConstraints, 0);
            },
            Set.of(this))
        .withName("Pathfind to Setup");
  }

  public Command pathFindForAlgaeRemover() {
    return new DeferredCommand(
            () -> {
              Pose2d closestPose;

              List<Pose2d> redAlgaeRemoverPoses = FieldConstants.redAlgaeRemoverPoses;
              List<Pose2d> blueAlgaeRemoverPoses = FieldConstants.blueAlgaeRemoverPoses;

              if (AllianceUtil.isRedAlliance()) {
                closestPose = stateCache.Pose.nearest(redAlgaeRemoverPoses);

              } else {
                closestPose = stateCache.Pose.nearest(blueAlgaeRemoverPoses);
              }

              return AutoBuilder.pathfindToPose(closestPose, AutoConstants.slowPathConstraints);
            },
            Set.of(this))
        .withName("Pathfind for Algae Remover");
  }

  public boolean seesTarget() {
    List<PhotonPipelineResult> latestResult = latestArducamFrontResult;

    if (latestResult == null || latestResult.isEmpty()) {
      return false;
    }

    for (PhotonPipelineResult result : latestResult) {
      if (!result.hasTargets()) continue;
      else {
        if (result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm() < 3) {
          return true;
        }
      }
    }
    return false;
  }

  public Command waitForTarget() {
    return Commands.waitUntil(() -> seesTarget()).withName("Wait for Target");
  }

  public boolean isAlgaeHighAtCurrentPosition() {

    Pose2d closestPose =
        AllianceUtil.isRedAlliance()
            ? stateCache.Pose.nearest(FieldConstants.redAlgaeRemoverPoses)
            : stateCache.Pose.nearest(FieldConstants.blueAlgaeRemoverPoses);

    return FieldConstants.algaeHeightPositions.getOrDefault(closestPose, false);
  }

  public double getAlgaeHeight() {
    return isAlgaeHighAtCurrentPosition()
        ? ElevatorConstants.AlgaeHighHeight
        : ElevatorConstants.AlgaeLowHeight;
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
    return new Pose3d(stateCache.Pose).plus(VisionConstants.arducamLeftTransform);
  }

  public Pose3d getArducamRightPose() {
    return new Pose3d(stateCache.Pose).plus(VisionConstants.arducamRightTransform);
  }

  public Pose3d getArducamFrontPose() {
    return new Pose3d(stateCache.Pose).plus(VisionConstants.arducamFrontTransform);
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

    // if (DriverStation.isAutonomous()) {
    //   return false;
    // }

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

    // if (DriverStation.isAutonomous()) {
    //   return false;
    // }

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
      Optional<Matrix<N3, N3>> cameraMatrix,
      Optional<Matrix<N8, N1>> distCoeffs,
      Transform3d cameraTransform,
      double baseSingleTagStdDev,
      double baseMultiTagStdDev,
      double cameraWeight) {
    if (latestResults.isEmpty()) {
      return;
    }

    poseEstimator.setReferencePose(stateCache.Pose);

    for (PhotonPipelineResult result : latestResults) {

      // List<PhotonTrackedTarget> filterTargets = result.getTargets().stream().filter(target->
      // !FieldConstants.rejectedTAGS.contains(target.getFiducialId())).toList();
      // if(filterTargets.isEmpty()) {
      //   continue;
      // }
      // PhotonPipelineResult filteredResult = new PhotonPipelineResult(filterTargets);

      Optional<EstimatedRobotPose> optionalVisionPose =
          poseEstimator.update(result, cameraMatrix, distCoeffs, constrainedPnpParams);
      if (optionalVisionPose.isEmpty()) {
        continue;
      }

      EstimatedRobotPose visionPose = optionalVisionPose.get();

      double totalDistance = 0.0;
      int tagCount = 0;

      for (PhotonTrackedTarget target : visionPose.targetsUsed) {
        // if (target.getFiducialId() == 5 ||target.getFiducialId() == 4 || target.getFiducialId()
        // == 15 || target.getFiducialId() == 14) {
        //   continue;
        // }
        tagCount++;
        totalDistance +=
            target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
      }

      double averageDistance = totalDistance / tagCount;

      if (tagCount > 1 && visionPose.strategy == poseEstimator.getPrimaryStrategy()) {
        if (!isValidMultitagPose(
            visionPose.estimatedPose,
            averageDistance,
            visionPose.targetsUsed.size(),
            visionPose.timestampSeconds)) {
          rejectedPoses.add(visionPose.estimatedPose);
          continue;
        }
        poseEstimates.add(
            new PoseEstimate(
                visionPose.estimatedPose,
                visionPose.timestampSeconds,
                getVisionStdDevs(
                    tagCount, averageDistance, baseMultiTagStdDev * (1 / cameraWeight))));
      } else {
        // PhotonTrackedTarget target = visionPose.targetsUsed.get(0);
        // Optional<Pose2d> robotPoseAtTime =
        //     samplePoseAt(Utils.fpgaToCurrentTime(visionPose.timestampSeconds));
        // Optional<Pose3d> tagOnField =
        //     FieldConstants.aprilTagLayout.getTagPose(target.getFiducialId());

        // if (robotPoseAtTime.isEmpty() || tagOnField.isEmpty()) {
        //   continue;
        // }

        // Pose3d singleTagPose =
        //     calculateSingleTagPose(
        //         target, tagOnField.get(), robotPoseAtTime.get(), cameraTransform);

        if (!isValidSingleTagPose(visionPose.estimatedPose, averageDistance)) {
          rejectedPoses.add(visionPose.estimatedPose);
          continue;
        }

        poseEstimates.add(
            new PoseEstimate(
                visionPose.estimatedPose,
                visionPose.timestampSeconds,
                getVisionStdDevs(
                    tagCount, averageDistance, baseSingleTagStdDev * (1 / cameraWeight))));
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
        arducamLeftMatrix,
        arducamLeftDistCoeffs,
        VisionConstants.arducamLeftTransform,
        Units.inchesToMeters(3.0),
        Units.inchesToMeters(2.5),
        0.25);
    updateVisionPoses(
        latestArducamRightResult,
        arducamRightPoseEstimator,
        arducamRightMatrix,
        arducamRightDistCoeffs,
        VisionConstants.arducamRightTransform,
        Units.inchesToMeters(3.0),
        Units.inchesToMeters(2.5),
        0.25);
    updateVisionPoses(
        latestArducamFrontResult,
        arducamFrontPoseEstimator,
        arducamFrontMatrix,
        arducamFrontDistCoeffs,
        VisionConstants.arducamFrontTransform,
        Units.inchesToMeters(3.0),
        Units.inchesToMeters(2.5),
        1);

    Collections.sort(poseEstimates);

    for (PoseEstimate poseEstimate : poseEstimates) {
      addVisionMeasurement(
          poseEstimate.estimatedPose().toPose2d(),
          Utils.fpgaToCurrentTime(poseEstimate.timestamp()),
          poseEstimate.standardDevs());
    }
  }

  public List<PhotonPipelineResult> getarducamFrontResults() {
    return latestArducamFrontResult;
  }

  public List<PhotonPipelineResult> getArducamLeftResults() {
    return latestArducamLeftResult;
  }

  public List<PhotonPipelineResult> getArducamRightResults() {
    return latestArducamRightResult;
  }

  public final void cancelCurrentCommand() {
    // runOnce(() -> {}).schedule();
  }

  public final String getAlertGroup() {
    return getName() + "/Alerts";
  }

  public void clearAlerts() {
    for (Alert alert : prematchAlerts) {
      alert.close();
    }

    prematchAlerts.clear();
  }

  private final void addAlert(Alert alert) {
    alert.set(true);
    prematchAlerts.add(alert);
  }

  public final void addInfo(String message) {
    addAlert(new Alert(getAlertGroup(), message, AlertType.kInfo));
  }

  public final void addWarning(String message) {
    addAlert(new Alert(getAlertGroup(), message, AlertType.kWarning));
  }

  public final void addError(String message) {
    addAlert(new Alert(getAlertGroup(), message, AlertType.kError));
    setSystemStatus("Pre-Match failed with reason: \"" + message + "\"");
    Elastic.sendNotification(
        new Elastic.Notification()
            .withLevel(NotificationLevel.ERROR)
            .withTitle("Swerve Pre-Match Failed")
            .withDescription(message)
            .withDisplaySeconds(5));
  }

  public final void setSystemStatus(String status) {
    systemStatus = status;
  }

  public final String getSystemStatus() {
    return systemStatus;
  }

  public final boolean containsErrors() {
    for (Alert alert : prematchAlerts) {
      if (alert.getType() == AlertType.kError) {
        return true;
      }
    }

    return false;
  }

  public Command buildPrematch() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  cancelCurrentCommand();
                  clearAlerts();
                  setSystemStatus("Running Pre-Match Check");
                }),
            getPrematchCheckCommand())
        .until(this::containsErrors)
        .finallyDo(
            (interrupted) -> {
              cancelCurrentCommand();

              if (interrupted && !containsErrors()) {
                addError("Pre-Match Interrpted");
              } else if (!interrupted && !containsErrors()) {
                setSystemStatus("Pre-Match Successful!");
                Elastic.sendNotification(
                    new Elastic.Notification()
                        .withLevel(NotificationLevel.INFO)
                        .withTitle("Swerve Pre-Match Successful")
                        .withDescription(
                            "Swerve Pre-Match was successful, good luck in the next match!")
                        .withDisplaySeconds(3.5));
              }
            });
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

  double ctreToFpgaTime(double timestamp) {
    return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + timestamp;
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();
    stateCache = getState();
    field.setRobotPose(stateCache.Pose);

    double stateTimestamp = ctreToFpgaTime(stateCache.Timestamp);

    arducamLeftPoseEstimator.addHeadingData(stateTimestamp, stateCache.Pose.getRotation());
    arducamRightPoseEstimator.addHeadingData(stateTimestamp, stateCache.Pose.getRotation());
    arducamFrontPoseEstimator.addHeadingData(stateTimestamp, stateCache.Pose.getRotation());

    if (arducamFrontMatrix.isEmpty()) {
      arducamFrontMatrix = arducamFront.getCameraMatrix();
    }
    if (arducamFrontDistCoeffs.isEmpty()) {
      arducamFrontDistCoeffs = arducamFront.getDistCoeffs();
    }

    if (arducamLeftMatrix.isEmpty()) {
      arducamLeftMatrix = arducamLeft.getCameraMatrix();
    }
    if (arducamLeftDistCoeffs.isEmpty()) {
      arducamLeftDistCoeffs = arducamLeft.getDistCoeffs();
    }

    if (arducamRightMatrix.isEmpty()) {
      arducamRightMatrix = arducamRight.getCameraMatrix();
    }
    if (arducamRightDistCoeffs.isEmpty()) {
      arducamRightDistCoeffs = arducamRight.getDistCoeffs();
    }

    latestArducamLeftResult = arducamLeft.getAllUnreadResults();
    latestArducamRightResult = arducamRight.getAllUnreadResults();
    latestArducamFrontResult = arducamFront.getAllUnreadResults();

    updateVisionPoseEstimates();
    updateBestAlignmentPose();
    SmartDashboard.putBoolean("Algae Height High?", isAlgaeHighAtCurrentPosition());

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
    Pose2d robotPose = stateCache.Pose;

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

  public Command getPrematchCheckCommand() {
    return Commands.sequence(
        // Check for hardware errors
        Commands.runOnce(
            () -> {
              seedFieldCentric();
            }),
        Commands.runOnce(
            () -> {
              if (Math.abs(
                      stateCache
                          .Pose
                          .getRotation()
                          .minus(getOperatorForwardDirection())
                          .getDegrees())
                  > 0.25) {
                addError("Gyro did not zero");
              }
            }),
        Commands.race(
            Commands.run(
                () ->
                    setControl(
                        fieldOriented
                            .withVelocityX(SwerveConstants.maxTranslationalSpeed)
                            .withVelocityY(0)
                            .withRotationalRate(0))),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      double forwardSpeed = stateCache.Speeds.vxMetersPerSecond;
                      if (forwardSpeed < 0) {
                        addError(
                            "Robot is driving in the wrong direction! Should be driving forward");
                      } else if (forwardSpeed < preMatchTranslationalTolerance) {
                        addError("Forward Speed is too slow");
                      } else if (Math.abs(stateCache.Speeds.vyMetersPerSecond)
                          > preMatchTranslationalTolerance) {
                        addError("Strafe Speed is too high");
                      } else {
                        addInfo("Forward Speed is good!");
                      }
                    }))),
        Commands.waitSeconds(2),
        Commands.runOnce(
            () -> {
              double speedMagnitude =
                  Math.hypot(
                      stateCache.Speeds.vxMetersPerSecond, stateCache.Speeds.vyMetersPerSecond);
              if (speedMagnitude > preMatchTranslationalTolerance) {
                addError("Translational Speeds are too high");
              } else {
                addInfo("Slow Down was successful");
              }
            }),
        Commands.race(
            Commands.run(
                () ->
                    setControl(
                        fieldOriented
                            .withVelocityX(SwerveConstants.maxTranslationalSpeed.unaryMinus())
                            .withVelocityY(0)
                            .withRotationalRate(0))),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      double backwardSpeed = stateCache.Speeds.vxMetersPerSecond;
                      if (backwardSpeed > 0) {
                        addError(
                            "Robot is driving in the wrong direction! Should be driving backwards");
                      } else if (backwardSpeed > -preMatchTranslationalTolerance) {
                        addError("Backward Speed is too slow");
                      } else if (Math.abs(stateCache.Speeds.vyMetersPerSecond)
                          > preMatchTranslationalTolerance) {
                        addError("Strafe Speed is too high");
                      } else {
                        addInfo("Backward Speed is good!");
                      }
                    }))),
        Commands.waitTime(PreMatchConstants.prematchDelayBetweenSteps),
        Commands.race(
            Commands.run(
                () ->
                    setControl(
                        fieldOriented
                            .withVelocityX(0)
                            .withVelocityY(SwerveConstants.maxTranslationalSpeed)
                            .withRotationalRate(0))),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      double strafeSpeed = stateCache.Speeds.vyMetersPerSecond;
                      if (strafeSpeed < 0) {
                        addError("Robot is driving in the wrong direction! Should be driving left");
                      } else if (strafeSpeed < preMatchTranslationalTolerance) {
                        addError("Left Speed is too slow");
                      } else if (Math.abs(stateCache.Speeds.vxMetersPerSecond)
                          > preMatchTranslationalTolerance) {
                        addError("Forward/Backward Speed is too high");
                      } else {
                        addInfo("Left Speed is good!");
                      }
                    }))),
        Commands.waitTime(PreMatchConstants.prematchDelayBetweenSteps),
        Commands.race(
            Commands.run(
                () ->
                    setControl(
                        fieldOriented
                            .withVelocityX(0)
                            .withVelocityY(SwerveConstants.maxTranslationalSpeed.unaryMinus())
                            .withRotationalRate(0))),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      double strafeSpeed = stateCache.Speeds.vyMetersPerSecond;
                      if (strafeSpeed > 0) {
                        addError(
                            "Robot is driving in the wrong direction! Should be driving right");
                      }
                      if (strafeSpeed > -preMatchTranslationalTolerance) {
                        addError("Right Speed is too slow");
                      } else if (Math.abs(stateCache.Speeds.vxMetersPerSecond)
                          > preMatchTranslationalTolerance) {
                        addError("Forward/Backward Speed is too high");
                      } else {
                        addInfo("Right Speed is good!");
                      }
                    }))),
        Commands.waitTime(PreMatchConstants.prematchDelayBetweenSteps),
        Commands.race(
            Commands.run(
                () ->
                    setControl(
                        fieldOriented
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(SwerveConstants.maxRotationalSpeed.unaryMinus()))),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      double rotationalSpeed = stateCache.Speeds.omegaRadiansPerSecond;
                      if (rotationalSpeed > 0) {
                        addError(
                            "Robot is spinning the wrong direction! Should be spinning clockwise");
                      } else if (stateCache.Speeds.omegaRadiansPerSecond
                          > Units.degreesToRadians(-2.5)) {
                        addError("CW Speed is too slow");
                      } else {
                        addInfo("CW Speed is good!");
                      }
                    }))),
        Commands.waitTime(PreMatchConstants.prematchDelayBetweenSteps),
        Commands.race(
            Commands.run(
                () ->
                    setControl(
                        fieldOriented
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(SwerveConstants.maxRotationalSpeed))),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      double rotationalSpeed = stateCache.Speeds.omegaRadiansPerSecond;
                      if (rotationalSpeed < 0) {
                        addError(
                            "Robot is spinning in the wrong direction. Should be spinning counter clockwise");
                      }
                      if (rotationalSpeed < Units.degreesToRadians(2.5)) {
                        addError("CCW Speed is too slow");
                      } else {
                        addInfo("CCW Speed is good!");
                      }
                    }))));
  }
}
