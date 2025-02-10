package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Constants {
  public static class SwerveConstants {

    public static final LinearVelocity maxTranslationalSpeed = FeetPerSecond.of(15);
    public static final LinearVelocity slowModeMaxTranslationalSpeed = FeetPerSecond.of(5);
    public static final AngularVelocity maxRotationalSpeed = RotationsPerSecond.of(1.5);

    public static final Time translationZeroToFull = Seconds.of(.5);
    public static final Time rotationZeroToFull = Seconds.of(0.25);

    public static final LinearAcceleration maxTransationalAcceleration =
        maxTranslationalSpeed.div(translationZeroToFull);
    public static final AngularAcceleration maxAngularAcceleration =
        maxRotationalSpeed.div(rotationZeroToFull);
<<<<<<< HEAD
=======

    public static final double TRACK_WIDTH = Units.inchesToMeters(30.0);
    public static final double WHEEL_BASE = Units.inchesToMeters(30.0);

    public static final double centerToBumber = Units.inchesToMeters(18.5);

    public static final Translation2d frontLeft =
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d frontRight =
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d backLeft =
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d backRight =
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    public static final Translation2d[] wheelLocations = {
      frontLeft, frontRight, backLeft, backRight
    };

    public static final PathConstraints pathConstraints =
        new PathConstraints(
            SwerveConstants.maxTranslationalSpeed.in(MetersPerSecond),
            SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond),
            SwerveConstants.maxRotationalSpeed.in(RadiansPerSecond),
            SwerveConstants.maxAngularAcceleration.in(RadiansPerSecondPerSecond));
>>>>>>> 53ba8ad3c654736782878faff127b18904812eb6
  }

  public static class AutoConstants {
    public static final PIDConstants translationPID = new PIDConstants(5.0, 0.0, 0.0); // 5
    public static final PIDConstants rotationPID = new PIDConstants(1.0, 0.0, 0.0); // 1
  }

  public static class VisionConstants {
    public static final Path apriltaglayout =
        Paths.get("src/main/java/frc/robot/utils/2025-reefscape.json");

    public static final String kCameraName = "YOUR CAMERA NAME";

    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
<<<<<<< HEAD
=======

    public static final String limelightName = "limelight";
    public static final String arducamLeftName = "Arducam_Left";
    public static final String arducamRightName = "Arducam_Right";

    public static final Transform3d arducamLeftTransform =
        new Transform3d(
            Units.inchesToMeters(-12.619),
            Units.inchesToMeters(-12.619),
            Units.inchesToMeters(5.143),
            new Rotation3d(
                0.0, Units.degreesToRadians(-25), Units.degreesToRadians(225))); // Pitch: 65

    public static final Transform3d arducamRightTransform =
        new Transform3d(
            Units.inchesToMeters(-12.619),
            Units.inchesToMeters(12.619),
            Units.inchesToMeters(5.143),
            new Rotation3d(0.0, Units.degreesToRadians(-25), Units.degreesToRadians(135)));

    public static final Transform3d limelightTransform =
        new Transform3d(
            Units.inchesToMeters(12.525),
            Units.inchesToMeters(.5),
            Units.inchesToMeters(4.423),
            new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(0)));

    public static final Transform2d limelightTransform2d =
        new Transform2d(
            limelightTransform.getX(),
            limelightTransform.getY(),
            limelightTransform.getRotation().toRotation2d());

    public static final int[] reefAprilTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

    public static final double loopPeriodSecs = 0.016;

    public static Transform3d rightArducamTransform;

    public static Transform3d leftArducamTransform;
>>>>>>> 53ba8ad3c654736782878faff127b18904812eb6
  }

  // .890 7.415
  public static class FieldConstants {
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
<<<<<<< HEAD
=======

    public static final Pose2d redStationLeft =
        new Pose2d(16.638, 0.645, Rotation2d.fromDegrees(0));
    public static final Pose2d redStationRight =
        new Pose2d(16.638, 7.415, Rotation2d.fromDegrees(0));
    public static final Pose2d blueStationLeft = new Pose2d(0.85, 7.415, Rotation2d.fromDegrees(0));
    public static final Pose2d blueStationRight =
        new Pose2d(0.85, 0.645, Rotation2d.fromDegrees(0));

    public static final List<Pose2d> redSetupPoses =
        List.of(
            new Pose2d(15.297, 4.019, Rotation2d.fromDegrees(180)), // 0
            new Pose2d(14.176, 5.96, Rotation2d.fromDegrees(-120)), // 60
            new Pose2d(11.934, 5.96, Rotation2d.fromDegrees(-60)), // 120
            new Pose2d(10.813, 4.019, Rotation2d.fromDegrees(0)), // 180
            new Pose2d(11.934, 2.0774, Rotation2d.fromDegrees(60)), // -120
            new Pose2d(14.176, 2.0774, Rotation2d.fromDegrees(120))); // -60

    public static final List<Pose2d> blueSetupPoses =
        List.of(
            new Pose2d(6.725, 4.019, Rotation2d.fromDegrees(180)), // 0
            new Pose2d(5.604, 5.961, Rotation2d.fromDegrees(-120)), // 60
            new Pose2d(3.362, 5.961, Rotation2d.fromDegrees(-60)), // 120
            new Pose2d(2.241, 4.019, Rotation2d.fromDegrees(0)), // 180
            new Pose2d(3.362, 2.078, Rotation2d.fromDegrees(60)), // -120
            new Pose2d(5.604, 2.078, Rotation2d.fromDegrees(120))); // -60

    public static final Pose2d reefBlueAlliance =
        new Pose2d(4.483, 4.019, Rotation2d.fromDegrees(0.0));
    public static final Pose2d reefRedAlliance =
        new Pose2d(13.055, 4.019, Rotation2d.fromDegrees(0));

    public static final Map<Integer, Double> aprilTagAngles = new HashMap<>();

    static {
      aprilTagAngles.put(6, 120.0);
      aprilTagAngles.put(7, 180.0);
      aprilTagAngles.put(8, -120.0);
      aprilTagAngles.put(9, -60.0);
      aprilTagAngles.put(10, 0.0);
      aprilTagAngles.put(11, 60.0);
      aprilTagAngles.put(17, 60.0);
      aprilTagAngles.put(18, 0.0);
      aprilTagAngles.put(19, -60.0);
      aprilTagAngles.put(20, -120.0);
      aprilTagAngles.put(21, 180.0);
      aprilTagAngles.put(22, 120.0);
    }

    public static final Map<Integer, Double> left_aprilTagOffsets = new HashMap<>();

    static {
      left_aprilTagOffsets.put(6, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(7, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(8, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(9, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(10, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(11, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(17, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(18, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(19, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(20, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(21, Units.inchesToMeters(6.488));
      left_aprilTagOffsets.put(22, Units.inchesToMeters(6.488));
    }

    public static final Map<Integer, Double> right_aprilTagOffsets = new HashMap<>();

    static {
      right_aprilTagOffsets.put(6, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(7, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(8, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(9, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(10, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(11, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(17, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(18, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(19, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(20, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(21, Units.inchesToMeters(-6.488));
      right_aprilTagOffsets.put(22, Units.inchesToMeters(-6.488));
    }

    public static class RedReefPoses {
      public static final Pose2d faceOneLeft = new Pose2d();
      public static final Pose2d faceOneRight = new Pose2d();
      public static final Pose2d faceTwoLeft = new Pose2d();
      public static final Pose2d faceTwoRight = new Pose2d();
      public static final Pose2d faceThreeLeft = new Pose2d();
      public static final Pose2d faceThreeRight = new Pose2d();
      public static final Pose2d faceFourLeft = new Pose2d();
      public static final Pose2d faceFourRight = new Pose2d();
      public static final Pose2d faceFiveLeft = new Pose2d();
      public static final Pose2d faceFiveRight = new Pose2d();
      public static final Pose2d faceSixLeft = new Pose2d();
      public static final Pose2d faceSixRight = new Pose2d();
    }

    public static class BlueReefPoses {
      public static final Pose2d faceOneLeft = new Pose2d();
      public static final Pose2d faceOneRight = new Pose2d();
      public static final Pose2d faceTwoLeft = new Pose2d();
      public static final Pose2d faceTwoRight = new Pose2d();
      public static final Pose2d faceThreeLeft = new Pose2d();
      public static final Pose2d faceThreeRight = new Pose2d();
      public static final Pose2d faceFourLeft = new Pose2d();
      public static final Pose2d faceFourRight = new Pose2d();
      public static final Pose2d faceFiveLeft = new Pose2d();
      public static final Pose2d faceFiveRight = new Pose2d();
      public static final Pose2d faceSixLeft = new Pose2d();
      public static final Pose2d faceSixRight = new Pose2d();
    }

    public static class ReefDefinitePoses {
      public static final List<Pose2d> blueReefDefiniteLeftPoses =
          List.of(
              new Pose2d(5.812, 3.854, Rotation2d.fromDegrees(180)),
              new Pose2d(5.021, 2.807, Rotation2d.fromDegrees(120)),
              new Pose2d(3.689, 2.957, Rotation2d.fromDegrees(60)),
              new Pose2d(3.155, 4.189, Rotation2d.fromDegrees(0)),
              new Pose2d(3.953, 5.250, Rotation2d.fromDegrees(-60)),
              new Pose2d(5.313, 5.093, Rotation2d.fromDegrees(-120)));

      public static final List<Pose2d> blueReefDefiniteRightPoses =
          List.of(
              new Pose2d(5.812, 4.175, Rotation2d.fromDegrees(180)),
              new Pose2d(5.278, 2.957, Rotation2d.fromDegrees(120)),
              new Pose2d(3.967, 2.800, Rotation2d.fromDegrees(60)),
              new Pose2d(3.170, 3.861, Rotation2d.fromDegrees(0)),
              new Pose2d(3.689, 5.086, Rotation2d.fromDegrees(-60)),
              new Pose2d(5.029, 5.236, Rotation2d.fromDegrees(-120)));

      public static final List<Pose2d> redReefDefiniteRightPoses =
          List.of(
              new Pose2d(14.385, 4.182, Rotation2d.fromDegrees(180)),
              new Pose2d(13.861, 2.964, Rotation2d.fromDegrees(120)),
              new Pose2d(12.557, 2.807, Rotation2d.fromDegrees(60)),
              new Pose2d(11.755, 3.847, Rotation2d.fromDegrees(0)),
              new Pose2d(12.258, 5.072, Rotation2d.fromDegrees(-60)),
              new Pose2d(13.568, 5.250, Rotation2d.fromDegrees(-120)));

      public static final List<Pose2d> redReefDefiniteLeftPoses =
          List.of(
              new Pose2d(14.385, 3.861, Rotation2d.fromDegrees(180)),
              new Pose2d(13.576, 2.821, Rotation2d.fromDegrees(120)),
              new Pose2d(12.258, 2.964, Rotation2d.fromDegrees(60)),
              new Pose2d(11.755, 4.175, Rotation2d.fromDegrees(0)),
              new Pose2d(12.557, 5.243, Rotation2d.fromDegrees(-60)),
              new Pose2d(13.861, 5.079, Rotation2d.fromDegrees(-120)));
    }
  }

  public static class GyroConstants {
    public static final int pigeonID = 35;
>>>>>>> 53ba8ad3c654736782878faff127b18904812eb6
  }

  public static class IntakeConstants {
    public static final int groundIntakeMotorID = 25;
    public static final int armIntakeMotorID = 21;
    public static final int indexerMotorID = 16;

<<<<<<< HEAD
    public static final int intakeLaserCanID = 14;
    public static final int outakeLaserCanID = 15;

    public static final double indexerMotorSpeed = .5;
=======
    public static final double indexerMotorSpeed = .85;
>>>>>>> bae50d8cd504db551bb672f071f787e27348d379
    public static final double groundIntakeMotorSpeed = .9;
    public static final double outakeSpeed = -0.9;

    public static final int indexerCurrentLimit = 30;
    public static final double indexerShutOffLimit = 45;

    public static final int groundIntakeCurrentLimit = 30;
    public static final double groundIntakeShutOffLimit = 45;
<<<<<<< HEAD
=======
    public static final int intakeCurrentLimit = 30;
    public static final double algaeIntakeShutoffCurrentLimit = 45.0;
  }

  public static class OuttakeConstants {
    public static final int outtakeMotorID = 18;
    public static final int outtakeCurrentLimit = 25;
    public static final int outtakeShutOffLimit = 25;
    public static final double outtakeSpeed = 0.85;

    public static final int outtakeLaserCanID = 19;
  }

  public static class AlgaeIntakeConstants {
    public static final int algaeIntakeMotorID = 20;
    public static final double algaeIntakeSpeed = .2;
>>>>>>> bae50d8cd504db551bb672f071f787e27348d379
  }

  public static class ElevatorConstants {
    public static final double elevatorGearRatio = 1.0 / 6.0;
    public static final double L4Height = 2; // meters
    public static final double elevatorWheelRadius = Units.inchesToMeters(1.75); // meters

<<<<<<< HEAD
    public static final int elevatorMainMotorID = 15;
    public static final int elevatorFollowerMotorID = 26;
    public static final int buttonSwitchID = 23;

    public static final double maxHeight = 1.447800;
    public static final double minHeight = 0.0;

    public static final double L4Position_inRotations = 1.523;
=======
    public static final int elevatorMainMotorID = 48;
    public static final int elevatorFollowerMotorID = 49;
    public static final int buttonSwitchID = 0;

    public static final double maxHeight = Units.inchesToMeters(28.3);
    public static final double minHeight = 0.0;

    public static final double L4Height = Units.inchesToMeters(28.25);
    public static final double L3Height = Units.inchesToMeters(20);
    public static final double L2Height = Units.inchesToMeters(10);
    public static final double downHeight = Units.inchesToMeters(0);
>>>>>>> bae50d8cd504db551bb672f071f787e27348d379

    public static final LinearVelocity maxVelocity = MetersPerSecond.of(2.26);
    public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(5);

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity.in(MetersPerSecond))
            .withMotionMagicAcceleration(maxAcceleration.in(MetersPerSecondPerSecond));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
<<<<<<< HEAD
            .withKS(0.35)
            .withKV(0.12)
            .withKA(0.01)
            .withKG(0.2)
            .withKP(4.8)
            .withKI(0)
            .withKD(0.1)
=======
            .withKS(0.01)
            .withKV(5.16) // 5.14
            .withKA(0.03) // .04
            .withKG(0.25) // .31
            .withKP(13)
            .withKI(0.0)
            .withKD(.25) // 1
>>>>>>> bae50d8cd504db551bb672f071f787e27348d379
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
<<<<<<< HEAD
        new FeedbackConfigs().withSensorToMechanismRatio(elevatorGearRatio * elevatorWheelRadius);
=======
        new FeedbackConfigs().withSensorToMechanismRatio(1 / sensorToMechanismRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive) // needs to spin left when wires up
            .withNeutralMode(NeutralModeValue.Brake);
    public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(maxHeight)
            .withForwardSoftLimitEnable(true);
    // .withReverseSoftLimitThreshold(minHeight)
    // .withReverseSoftLimitEnable(true);

    public static final HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs =
        new HardwareLimitSwitchConfigs()
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitAutosetPositionValue(0)
            .withReverseLimitRemoteSensorID(ElevatorConstants.buttonSwitchID);
>>>>>>> 53ba8ad3c654736782878faff127b18904812eb6

    public static final TalonFXConfiguration elevatorConfigs =
        new TalonFXConfiguration()
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
<<<<<<< HEAD
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(maxHeight)
                    .withForwardSoftLimitEnable(true));
=======
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);
    // .withHardwareLimitSwitch(hardwareLimitSwitchConfigs);
  }
>>>>>>> 53ba8ad3c654736782878faff127b18904812eb6

<<<<<<< HEAD
    public static final SoftwareLimitSwitchConfigs limitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(maxHeight)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(minHeight)
            .withReverseSoftLimitEnable(true);
=======
  public static class ArmConstants {
    public static final int armMotorID = 47;
    public static final int armMaxVelocity = 0;
    public static final int armMaxAcceleration = 0;

    public static final int armCurrentLimit = 30;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(armMaxVelocity, armMaxAcceleration);
>>>>>>> bae50d8cd504db551bb672f071f787e27348d379
  }

  public static class OperatorConstants {
    public static final int indexerButton = 10;
    public static final int groundIntakeButton = 9;
    public static final int L4HeightButton = 8;
    public static final int homeElevatorButon = 7;
    public static final int manualOutakeButton = 6;
    public static final int manualFeedButton = 5;
  }
}
