package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
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
  }

  public static class FieldConstants {
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  }

  public static class IntakeConstants {
    public static final int groundIntakeMotorID = 25;
    public static final int armIntakeMotorID = 21;
    public static final int indexerMotorID = 22;

    public static final int intakeLaserCanID = 14;
    public static final int outakeLaserCanID = 15;

    public static final double indexerMotorSpeed = .5;
    public static final double groundIntakeMotorSpeed = .9;
    public static final double outakeSpeed = -0.9;

    public static final int indexerCurrentLimit = 30;
    public static final double indexerShutOffLimit = 45;

    public static final int groundIntakeCurrentLimit = 30;
    public static final double groundIntakeShutOffLimit = 45;
  }

  public static class ElevatorConstants {
    public static final double elevatorGearRatio = 1.0 / 6.0;
    public static final double L4Height = 2; // meters
    public static final double elevatorWheelRadius = Units.inchesToMeters(1.75); // meters

    public static final int elevatorMainMotorID = 15;
    public static final int elevatorFollowerMotorID = 26;
    public static final int buttonSwitchID = 23;

    public static final double maxHeight = 1.447800;
    public static final double minHeight = 0.0;

    public static final double L4Position_inRotations = 1.523;

    public static final LinearVelocity maxVelocity = MetersPerSecond.of(2.26);
    public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(5);

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity.in(MetersPerSecond))
            .withMotionMagicAcceleration(maxAcceleration.in(MetersPerSecondPerSecond));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.35)
            .withKV(0.12)
            .withKA(0.01)
            .withKG(0.2)
            .withKP(4.8)
            .withKI(0)
            .withKD(0.1)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(elevatorGearRatio * elevatorWheelRadius);

    public static final TalonFXConfiguration elevatorConfigs =
        new TalonFXConfiguration()
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(maxHeight)
                    .withForwardSoftLimitEnable(true));

    public static final SoftwareLimitSwitchConfigs limitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(maxHeight)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(minHeight)
            .withReverseSoftLimitEnable(true);
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
