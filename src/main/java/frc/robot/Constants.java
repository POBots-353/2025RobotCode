package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
        AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public static final Transform3d kRobotToCam =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final String limelightName = "limelight";
    public static final String arducamOneName = "Arducam_OV9281";
    public static final String arducamTwoName = "Arducam_OVO2";

    public static final Transform3d arducamOneTransform =
        new Transform3d(
            Units.inchesToMeters(-4.5),
            Units.inchesToMeters(-13.25),
            Units.inchesToMeters(8.50),
            new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(180.0)));

    public static final Transform3d arducamTwoTransform =
        new Transform3d(
            Units.inchesToMeters(-4.5),
            Units.inchesToMeters(-13.25),
            Units.inchesToMeters(8.50),
            new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(180.0)));

    public static final Transform3d limelightTransform =
        new Transform3d(
            Units.inchesToMeters(-4.5),
            Units.inchesToMeters(-13.25),
            Units.inchesToMeters(8.50),
            new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(180.0)));
  }

  public static class FieldConstants {
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // NEED TO CHANGE THESE VALUES
    public static final Pose2d reefBlueAlliance = new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0.0));
    public static final Pose2d reefRedAlliance =
        new Pose2d(16.54, 5.5, Rotation2d.fromDegrees(180.0));
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
    public static final int outtakeButton = 5;

    public static final int outtakeLaserCanID = 15;
  }

  public static class AlgaeIntakeConstants {
    public static final int algaeIntakeMotorID = 25;
    public static final double algaeIntakeSpeed = .5;
  }

  public static class ElevatorConstants {
    public static final double elevatorGearRatio = 1.0 / 6.0;
    public static final double L4Height = 15; // L4Height = 1.3; // meters
    public static final double elevatorWheelRadius = Units.inchesToMeters(1.75); // meters

    public static final int elevatorMainMotorID = 33;
    public static final int elevatorFollowerMotorID = 26;
    public static final int buttonSwitchID = 23;

    public static final double maxHeight = 1.447800;
    public static final double minHeight = 0.0;

    public static final double L4Position_inRotations = 1.523;

    public static final double bottomSpeed = .1;

    public static final LinearVelocity maxVelocity = MetersPerSecond.of(30);
    public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(20);

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity.in(MetersPerSecond))
            .withMotionMagicAcceleration(maxAcceleration.in(MetersPerSecondPerSecond));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(0.0)
            .withKP(10.0)
            .withKI(0.0)
            .withKD(0.1)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    // public static final FeedbackConfigs feedbackConfigs =
    //     new FeedbackConfigs().withSensorToMechanismRatio(elevatorGearRatio *
    // elevatorWheelRadius);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final TalonFXConfiguration elevatorConfigs =
        new TalonFXConfiguration()
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            // .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(40)
                    .withForwardSoftLimitEnable(true));
  }

  public static class OperatorConstants {
    public static final int indexerButton = 10;
    public static final int groundIntakeButton = 9;
    public static final int L4HeightButton = 1;
    public static final int homeElevatorButon = 7;
    public static final int manualOuttakeButton = 6;
    public static final int manualFeedButton = 14;
  }
}
