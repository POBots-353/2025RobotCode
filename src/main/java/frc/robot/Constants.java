package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import java.nio.file.Path;
import java.nio.file.Paths;

public class Constants {
  public static class SwerveConstants {
    public static final LinearVelocity maxTranslationalSpeed = FeetPerSecond.of(15);
    public static final LinearVelocity slowModeMaxTranslationalSpeed = FeetPerSecond.of(5);
    public static final AngularVelocity maxRotationalSpeed = RotationsPerSecond.of(1);

    public static final Time translationZeroToFull = Seconds.of(.5);
    public static final Time rotationZeroToFull = Seconds.of(0.25);

    public static final LinearAcceleration maxTransationalAcceleration =
        maxTranslationalSpeed.div(translationZeroToFull);
    public static final AngularAcceleration maxAngularAcceleration =
        maxRotationalSpeed.div(rotationZeroToFull);

    public static final double TRACK_WIDTH = Units.inchesToMeters(30.0);
    public static final double WHEEL_BASE = Units.inchesToMeters(30.0);

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
  }

  public static class AutoConstants {
    public static final PIDConstants translationPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationPID = new PIDConstants(1.0, 0.0, 0.0);
  }

  public static class VisionConstants {
    public static final Path apriltaglayout =
        Paths.get("src/main/java/frc/robot/utils/2025-reefscape.json");

    public static final String kCameraName = "YOUR CAMERA NAME";

    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final String limelightName = "limelight";
    public static final String arducamLeftName = "Arducam_OV9281";
    public static final String arducamRightName = "Arducam_OVO2";

    public static final double aprilTagReefOffset = Units.inchesToMeters(6.488);

    public static final Transform3d arducamLeftTransform =
        new Transform3d(
            Units.inchesToMeters(-12.619),
            Units.inchesToMeters(-12.619),
            Units.inchesToMeters(5.143),
            new Rotation3d(
                0.0, Units.degreesToRadians(0), Units.degreesToRadians(225))); // Pitch: 65

    public static final Transform3d arducamRightTransform =
        new Transform3d(
            Units.inchesToMeters(-12.619),
            Units.inchesToMeters(12.619),
            Units.inchesToMeters(5.143),
            new Rotation3d(0.0, Units.degreesToRadians(0), Units.degreesToRadians(135)));

    public static final Transform3d limelightTransform =
        new Transform3d(
            Units.inchesToMeters(12.525),
            Units.inchesToMeters(0),
            Units.inchesToMeters(4.423),
            new Rotation3d(0.0, Units.degreesToRadians(0), Units.degreesToRadians(0)));

    public static final Transform2d limelightTransform2d =
        new Transform2d(
            limelightTransform.getX(),
            limelightTransform.getY(),
            limelightTransform.getRotation().toRotation2d());

    public static final int[] reefAprilTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

    public static final double loopPeriodSecs = 0.016;

    public static Transform3d rightArducamTransform;

    public static Transform3d leftArducamTransform;
  }

  public static class FieldConstants {
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // NEED TO CHANGE THESE VALUES
    public static final Pose2d reefBlueAlliance = new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0.0));
    public static final Pose2d reefRedAlliance =
        new Pose2d(16.54, 5.5, Rotation2d.fromDegrees(180.0));

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
  }

  public static class GyroConstants {
    public static final int pigeonID = 35;
  }

  public static class IntakeConstants {
    public static final int groundIntakeMotorID = 23;
    public static final int armIntakeMotorID = 21;
    public static final int indexerMotorID = 22;

    public static final int intakeLaserCanID = 14;

    public static final double indexerMotorSpeed = .5;
    public static final double groundIntakeMotorSpeed = .9;
    public static final double outtakeSpeed = -0.9;

    public static final int indexerCurrentLimit = 30;
    public static final double indexerShutOffLimit = 45;

    public static final int groundIntakeCurrentLimit = 30;
    public static final double groundIntakeShutOffLimit = 45;
    public static final int intakeCurrentLimit = 30;
    public static final double algaeIntakeShutoffCurrentLimit = 45.0;
  }

  public static class OuttakeConstants {
    public static final int outtakeMotorID = 46;
    public static final int outtakeCurrentLimit = 25;
    public static final int outtakeShutOffLimit = 25;
    public static final double outtakeSpeed = 0.5;

    public static final int outtakeLaserCanID = 15;
  }

  public static class AlgaeIntakeConstants {
    public static final int algaeIntakeMotorID = 25;
    public static final double algaeIntakeSpeed = .5;
  }

  public static class ElevatorConstants {
    public static final double elevatorGearRatio = 1.0 / 6.0;
    public static final double sprocketDiameter = Units.inchesToMeters(1.75);

    public static final int elevatorMainMotorID = 48;
    public static final int elevatorFollowerMotorID = 49;
    public static final int buttonSwitchID = 23;

    public static final double maxHeight = Units.inchesToMeters(27.5);
    public static final double minHeight = 0.0;

    public static final double L4Height = Units.inchesToMeters(27.25);
    public static final double L3Height = Units.inchesToMeters(20);
    public static final double L2Height = Units.inchesToMeters(10);
    public static final double downHeight = Units.inchesToMeters(0);

    public static final double sensorToMechanismRatio =
        elevatorGearRatio * Math.PI * sprocketDiameter;

    public static final double bottomSpeed = .1;

    public static final LinearVelocity maxVelocity = MetersPerSecond.of(2.26 * .9); // 2.26*.9
    public static final LinearAcceleration maxAcceleration = maxVelocity.div(Seconds.of(.5)); // .25

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity.in(MetersPerSecond))
            .withMotionMagicAcceleration(maxAcceleration.in(MetersPerSecondPerSecond));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.01)
            .withKV(1.72) // 5.14
            .withKA(0.01) // .04
            .withKG(0.1) // .31
            .withKP(20)
            .withKI(0.0)
            .withKD(.25) // 1
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(1 / sensorToMechanismRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive) // needs to spin left when wires up
            .withNeutralMode(NeutralModeValue.Brake);
    public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(maxHeight)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(minHeight)
            .withReverseSoftLimitEnable(true);

    public static final HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs =
        new HardwareLimitSwitchConfigs()
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitAutosetPositionValue(0)
            .withReverseLimitRemoteSensorID(ElevatorConstants.buttonSwitchID);

    public static final TalonFXConfiguration elevatorConfigs =
        new TalonFXConfiguration()
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
            .withHardwareLimitSwitch(hardwareLimitSwitchConfigs);
  }

  public static class ArmConstants {
    public static final int armMaxVelocity = 0;
    public static final int armMaxAcceleration = 0;

    public static final int armCurrentLimit = 30;

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(armMaxVelocity, armMaxAcceleration);
  }

  public static class OperatorConstants {
    public static final int indexerButton = 13;
    public static final int outtakeIndexerButton = 4;

    public static final int groundIntakeButton = 13;
    public static final int armManualOuttakeButton = 12;

    public static final int armPickupHeightButton = 5;
    public static final int armL1HeightButton = 6;
    public static final int armManualUp = 10;
    public static final int armManualDown = 9;

    public static final int L4HeightButton = 8;
    public static final int L3HeightButton = 7;
    public static final int L2HeightButton = 6;
    public static final int elevatorDownButton = 5;
    public static final int elevatorManualUp = 10;
    public static final int elevatorManualDown = 9;
    public static final int homeElevatorButon = 2;

    public static final int outtakeButton = 12;

    public static final int algaeIntakeDown = 14;
    public static final int algaeIntakeUp = 15;

    public static final int armModeButton = 16;

    public static final int startingConfigButton = 11;
  }
}
