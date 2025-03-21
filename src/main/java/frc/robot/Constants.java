package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Constants {
  public static class SwerveConstants {

    public static final LinearVelocity maxTranslationalSpeed = FeetPerSecond.of(15);
    public static final LinearVelocity slowModeMaxTranslationalSpeed = FeetPerSecond.of(5);
    public static final AngularVelocity maxRotationalSpeed = RotationsPerSecond.of(1.5);

    public static final Time translationZeroToFull = Seconds.of(0.6);
    public static final Time rotationZeroToFull = Seconds.of(0.25);

    public static final LinearAcceleration maxTransationalAcceleration =
        maxTranslationalSpeed.div(translationZeroToFull);
    public static final AngularAcceleration maxAngularAcceleration =
        maxRotationalSpeed.div(rotationZeroToFull);

    public static final double centerToBumber = Units.inchesToMeters(18);
  }

  public static class AutoConstants {
    public static final PIDConstants translationPID = new PIDConstants(2, 0.0, 0.1); // 5 2.2
    public static final PIDConstants rotationPID = new PIDConstants(1.4, 0.0, 0.1); // 1  2.8

    public static final LinearVelocity autoMaxTranslationalSpeed = FeetPerSecond.of(15);
    public static final AngularVelocity autoMaxRotationalSpeed = RotationsPerSecond.of(1.0);

    public static final LinearAcceleration autoMaxTransationalAcceleration =
        // autoMaxTranslationalSpeed.div(SwerveConstants.translationZeroToFull);
        autoMaxTranslationalSpeed.div(Seconds.of(.3));

    public static final AngularAcceleration autoMaxAngularAcceleration =
        autoMaxRotationalSpeed.div(SwerveConstants.rotationZeroToFull);

    public static final PathConstraints fastPathConstraints =
        new PathConstraints(
            autoMaxTranslationalSpeed.in(MetersPerSecond),
            autoMaxTransationalAcceleration.in(MetersPerSecondPerSecond),
            autoMaxRotationalSpeed.in(RadiansPerSecond),
            autoMaxAngularAcceleration.in(RadiansPerSecondPerSecond));

    public static final PathConstraints HPPathConstraints =
        new PathConstraints(
            3.7,
            10,
            autoMaxRotationalSpeed.in(RadiansPerSecond),
            autoMaxAngularAcceleration.in(RadiansPerSecondPerSecond));

    public static final PathConstraints midPathConstraints = new PathConstraints(3.53, 6.5, 7, 13);

    public static final PathConstraints slowPathConstraints =
        new PathConstraints(2.5, 2.5, Units.degreesToRadians(180), Units.degreesToRadians(360));
  }

  public static class VisionConstants {
    public static final String arducamFrontName = "Arducam_Front";
    public static final String arducamLeftName = "Arducam_Left";
    public static final String arducamRightName = "Arducam_Right";

    public static final Transform3d arducamLeftTransform =
        new Transform3d(
            Units.inchesToMeters(-13.109895),
            Units.inchesToMeters(13.165),
            Units.inchesToMeters(8.079),
            new Rotation3d(
                0, Units.degreesToRadians(-25), Units.degreesToRadians(180 - 45))); // Pitch: 65

    public static final Transform3d arducamRightTransform =
        new Transform3d(
            Units.inchesToMeters(-13.102380),
            Units.inchesToMeters(-13.402856),
            Units.inchesToMeters(8.079),
            new Rotation3d(0.0, Units.degreesToRadians(-25), Units.degreesToRadians(180 + 45)));

    public static final Transform3d arducamFrontTransform =
        new Transform3d(
            Units.inchesToMeters(13.778840),
            Units.inchesToMeters(-0.105402),
            Units.inchesToMeters(7.800704),
            new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(0)));

    public static final Transform2d arducamFrontTransform2d =
        new Transform2d(
            arducamFrontTransform.getX(),
            arducamFrontTransform.getY(),
            arducamFrontTransform.getRotation().toRotation2d());

    public static final int[] reefAprilTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  }

  // .890 7.415
  public static class FieldConstants {
    public static final List<Integer> rejectedTAGS = List.of(4, 14, 15, 5);
    public static final String aprilTagJson = "2025-official-welded";
    public static final Path aprilTagJsonPath =
        Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", aprilTagJson + ".json");

    public static AprilTagFieldLayout aprilTagLayout;

    static {
      try {
        aprilTagLayout = new AprilTagFieldLayout(aprilTagJsonPath);
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }

    // public static final Pose2d redStationLeft =
    //     new Pose2d(15.980, 0.686, Rotation2d.fromDegrees(126.87));
    // public static final Pose2d redStationRight =
    //     new Pose2d(15.980, 7.400, Rotation2d.fromDegrees(-126.87));
    // public static final Pose2d blueStationLeft =
    //     new Pose2d(1.58, 7.400, Rotation2d.fromDegrees(-53.130));
    // public static final Pose2d blueStationRight =
    //     new Pose2d(1.58, 0.686, Rotation2d.fromDegrees(53.130));

    public static final Pose2d redStationLeft =
        new Pose2d(15.826813936233481, 1.0288156509399498, Rotation2d.fromDegrees(125.1));
    public static final Pose2d redStationRight =
        new Pose2d(15.726813936233482, 7.128815650939941, Rotation2d.fromDegrees(-125.1));
    public static final Pose2d blueStationLeft =
        new Pose2d(1.8268139362335205, 7.128815650939941, Rotation2d.fromDegrees(-53.4));
    public static final Pose2d blueStationRight =
        new Pose2d(1.667565608024597, 1.0683921813964927, Rotation2d.fromDegrees(53.4));

    public static final Pose2d blueBargePose =
        new Pose2d(7.419384479522705, 5.5195770263671875, Rotation2d.kZero);
    public static final Pose2d redBargePose =
        new Pose2d(10.136296272277832, 2.426062822341919, Rotation2d.k180deg);

    public static final Pose2d blueProcessorPose = new Pose2d(6.011, 1.0, Rotation2d.kCW_90deg);
    public static final Pose2d redProcessorPose = new Pose2d(11.524, 7.0, Rotation2d.kCCW_90deg);

    public static final List<Pose2d> redSetupPoses =
        List.of(
            new Pose2d(10.813, 4.019, Rotation2d.fromDegrees(0)), // 180
            new Pose2d(11.934, 2.0774, Rotation2d.fromDegrees(60)), // -120
            new Pose2d(14.176, 2.0774, Rotation2d.fromDegrees(120)), // -60
            new Pose2d(15.297, 4.019, Rotation2d.fromDegrees(180)), // 0
            new Pose2d(14.176, 5.96, Rotation2d.fromDegrees(-120)), // 60
            new Pose2d(11.934, 5.96, Rotation2d.fromDegrees(-60))); // 120

    public static final List<Pose2d> blueSetupPoses =
        List.of(
            new Pose2d(6.725, 4.019, Rotation2d.fromDegrees(180)), // 0
            new Pose2d(5.604, 5.961, Rotation2d.fromDegrees(-120)), // 60
            new Pose2d(3.362, 5.961, Rotation2d.fromDegrees(-60)), // 120
            new Pose2d(2.241, 4.019, Rotation2d.fromDegrees(0)), // 180
            new Pose2d(3.362, 2.078, Rotation2d.fromDegrees(60)), // -120
            new Pose2d(5.604, 2.078, Rotation2d.fromDegrees(120))); // -60

    public static final List<Pose2d> blueAlgaeRemoverPoses =
        List.of(
            new Pose2d(5.803, 4.049, Rotation2d.fromDegrees(180)), // 0
            new Pose2d(5.137, 5.167, Rotation2d.fromDegrees(-120)), // 60
            new Pose2d(3.835, 5.160, Rotation2d.fromDegrees(-60)), // 120
            new Pose2d(3.182, 4.028, Rotation2d.fromDegrees(0)), // 180
            new Pose2d(3.832, 2.890, Rotation2d.fromDegrees(60)), // -120
            new Pose2d(5.150, 2.896, Rotation2d.fromDegrees(120))); // -60

    public static final List<Pose2d> redAlgaeRemoverPoses =
        List.of(
            new Pose2d(14.377, 4.019, Rotation2d.fromDegrees(180)), // 0
            new Pose2d(13.719, 5.162, Rotation2d.fromDegrees(-120)), // 60
            new Pose2d(12.415, 5.150, Rotation2d.fromDegrees(-60)), // 120
            new Pose2d(11.759, 4.018, Rotation2d.fromDegrees(0)), // 180
            new Pose2d(12.415, 2.893, Rotation2d.fromDegrees(60)), // -120
            new Pose2d(13.725, 2.907, Rotation2d.fromDegrees(120))); // -60

    public static final Map<Pose2d, Boolean> algaeHeightPositions = new HashMap<>();

    static {
      // blue
      algaeHeightPositions.put(new Pose2d(5.803, 4.049, Rotation2d.fromDegrees(180)), false);
      algaeHeightPositions.put(new Pose2d(5.137, 5.167, Rotation2d.fromDegrees(-120)), true);
      algaeHeightPositions.put(new Pose2d(3.835, 5.160, Rotation2d.fromDegrees(-60)), false);
      algaeHeightPositions.put(new Pose2d(3.182, 4.028, Rotation2d.fromDegrees(0)), true);
      algaeHeightPositions.put(new Pose2d(3.832, 2.890, Rotation2d.fromDegrees(60)), false);
      algaeHeightPositions.put(new Pose2d(5.150, 2.896, Rotation2d.fromDegrees(120)), true);

      // red
      algaeHeightPositions.put(new Pose2d(14.377, 4.019, Rotation2d.fromDegrees(180)), true);
      algaeHeightPositions.put(new Pose2d(13.719, 5.162, Rotation2d.fromDegrees(-120)), false);
      algaeHeightPositions.put(new Pose2d(12.415, 5.150, Rotation2d.fromDegrees(-60)), true);
      algaeHeightPositions.put(new Pose2d(11.759, 4.018, Rotation2d.fromDegrees(0)), false);
      algaeHeightPositions.put(new Pose2d(12.415, 2.893, Rotation2d.fromDegrees(60)), true);
      algaeHeightPositions.put(new Pose2d(13.725, 2.907, Rotation2d.fromDegrees(120)), false);
    }

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
              new Pose2d(5.803, 3.858, Rotation2d.fromDegrees(180)),
              new Pose2d(4.999, 2.806, Rotation2d.fromDegrees(120)),
              new Pose2d(3.691, 2.975, Rotation2d.fromDegrees(60)),
              new Pose2d(3.181, 4.187, Rotation2d.fromDegrees(0)),
              new Pose2d(3.975, 5.244, Rotation2d.fromDegrees(-60)),
              new Pose2d(5.279, 5.077, Rotation2d.fromDegrees(-120)));

      public static final List<Pose2d> blueReefDefiniteRightPoses =
          List.of(
              new Pose2d(5.803, 4.187, Rotation2d.fromDegrees(180)),
              new Pose2d(5.288, 2.970, Rotation2d.fromDegrees(120)),
              new Pose2d(3.976, 2.807, Rotation2d.fromDegrees(60)),
              new Pose2d(3.186, 3.859, Rotation2d.fromDegrees(0)),
              new Pose2d(3.690, 5.077, Rotation2d.fromDegrees(-60)),
              new Pose2d(4.993, 5.245, Rotation2d.fromDegrees(-120)));

      public static final List<Pose2d> redReefDefiniteRightPoses =
          List.of(
              new Pose2d(14.372, 4.189, Rotation2d.fromDegrees(180)),
              new Pose2d(13.858, 2.976, Rotation2d.fromDegrees(120)),
              new Pose2d(12.553, 2.808, Rotation2d.fromDegrees(60)),
              new Pose2d(11.755, 3.849, Rotation2d.fromDegrees(0)),
              new Pose2d(12.267, 5.078, Rotation2d.fromDegrees(-60)),
              new Pose2d(13.572, 5.238, Rotation2d.fromDegrees(-120)));

      public static final List<Pose2d> redReefDefiniteLeftPoses =
          List.of(
              new Pose2d(14.373, 3.860, Rotation2d.fromDegrees(180)),
              new Pose2d(13.578, 2.814, Rotation2d.fromDegrees(120)),
              new Pose2d(12.269, 2.980, Rotation2d.fromDegrees(60)),
              new Pose2d(11.755, 4.189, Rotation2d.fromDegrees(0)),
              new Pose2d(12.553, 5.243, Rotation2d.fromDegrees(-60)),
              new Pose2d(13.854, 5.076, Rotation2d.fromDegrees(-120)));
    }
  }

  public static class IntakeConstants {
    public static final int groundIntakeMotorID = 14;
    public static final int indexerMotorID = 16;
    public static final int indexerFollowerMotorID = 40;

    public static final double indexerMotorSpeed = .85;
    public static final double groundIntakeMotorSpeed = .9;
    public static final double outtakeSpeed = -.7;

    public static final int indexerCurrentLimit = 35;
    public static final double indexerShutOffLimit = 60;

    public static final int groundIntakeCurrentLimit = 30;
    public static final double groundIntakeShutOffLimit = 45;
    public static final int intakeCurrentLimit = 30;
  }

  public static class OuttakeConstants {
    public static final int outtakeMotorID = 18;
    public static final int outtakeCurrentLimit = 60;
    public static final int outtakeShutOffLimit = 75;

    public static final double fastOuttakeSpeed = 0.40;
    public static final double slowOuttakeSpeed = 0.24; // .353 0.2

    public static final int outtakeLaserCanID = 19;
  }

  public static class AlgaeRemoverConstants {
    public static final int algaeRemoverMotorID = 17;
    public static final int algaeIntakeMotorID = 20;

    public static final double planetaryGearRatio = 1.0 / 125.0;
    public static final double outputGearRatio = 18.0 / 32.0;

    public static final double absoluteEncoderConversion = 2 * Math.PI * outputGearRatio;
    public static final double internalEncoderConversion =
        2 * Math.PI * planetaryGearRatio * outputGearRatio;

    public static final double algaeRemoverSpeed = .5;
    public static final double algaeIntakeSpeed = 1.0;
    public static final double slowAlgaeIntakeSpeed = .1;

    public static final int currentLimit = 50;
    public static final int shutOffCurrentLimit = 80;

    public static final Angle intakePosition = Degrees.of(-82.0);
    public static final Angle stowPosition = Degrees.of(15.0);
    public static final Angle bargePosition = Degrees.of(-31.0);
    public static final Angle processorPosition = Degrees.of(-45);
    public static final Angle floorAlgaePosition = Degrees.of(-172.0);
    public static final Angle holdPosition = Degrees.of(-4.0);

    public static final Angle maxPosition = Degrees.of(0);
    public static final Angle minPosition = Degrees.of(-184);

    public static final Time reefIntakeTimingOffset = Seconds.of(.6);

    public static final double maxVelocity = Units.degreesToRadians(30.0); // Degrees per second
    public static final double maxAcceleration =
        Units.degreesToRadians(50.0); // Degrees per second squared

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true);

    public static final TalonFXConfiguration algaeIntakeConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withMotorOutput(motorOutputConfigs);
  }

  public static class ElevatorConstants {
    public static final double elevatorGearRatio = 6.0 / 1.0;
    public static final double sprocketDiameter = Units.inchesToMeters(1.75);

    public static final int elevatorMainMotorID = 21;
    public static final int elevatorFollowerMotorID = 22;
    public static final int buttonSwitchID = 0;

    public static final double maxHeight = Units.inchesToMeters(28.353);
    public static final double minHeight = 0.0;

    public static final double L4Height = Units.inchesToMeters(28.3); // 28.09
    public static final double L3Height = Units.inchesToMeters(15.5 + .7);
    public static final double L2Height = Units.inchesToMeters(7.3 + .55);
    public static final double downHeight = Units.inchesToMeters(0);
    public static final double bargeHeight = Units.inchesToMeters(28);
    public static final double processorHeight = Units.inchesToMeters(2);
    public static final double floorAlgaeHeight = Units.inchesToMeters(5);

    public static final double coralInTheWayAdd = 2.74;

    public static final double AlgaeHighHeight = Units.inchesToMeters(9.2);
    public static final double AlgaeLowHeight = Units.inchesToMeters(1.5);

    public static final double sensorToMechanismRatio =
        elevatorGearRatio / (sprocketDiameter * Math.PI);

    public static final double bottomSpeed = 0.1;

    public static final LinearVelocity maxVelocity = MetersPerSecond.of(2.26 * 0.95); // 2.26*.9
    public static final LinearAcceleration maxAcceleration =
        maxVelocity.div(Seconds.of(0.5)); // .25

    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity.in(MetersPerSecond))
            .withMotionMagicAcceleration(maxAcceleration.in(MetersPerSecondPerSecond));

    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.02) // .01
            .withKV(4.14) // 4.14
            .withKA(0.1) // .03
            .withKG(0.33) // .31
            .withKP(35) // 28
            .withKI(0.0)
            .withKD(0.005) // .11
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechanismRatio);

    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive) // needs to spin left when wires up
            .withNeutralMode(NeutralModeValue.Brake);
    public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(maxHeight)
            .withForwardSoftLimitEnable(true);
    public static final CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs().withStatorCurrentLimit(45).withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration elevatorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);
  }

  public static class ArmConstants {
    public static final int armMotorID = 15;
    public static final int armMaxVelocity = 50; // degrees/s
    public static final int armMaxAcceleration = 100; // degrees/s^2

    public static final int armCurrentLimit = 30;

    public static final Rotation2d armTopPosition = Rotation2d.fromDegrees(85);
    public static final Rotation2d armL1Position = Rotation2d.fromDegrees(130);
    public static final Rotation2d armBottomPosition = Rotation2d.fromDegrees(30);

    public static final double downHeight = Units.inchesToMeters(0);

    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(armMaxVelocity, armMaxAcceleration);

    public static final double replanningError = Units.degreesToRadians(55.0);
    public static final double angleTolerance = Units.degreesToRadians(0.15);

    public static double armKg = 0;
    public static double armKs = 0;
    public static double armKv = 0;
    public static double armKa = 0;
    public static double armKp = 0;
    public static double armKi = 0;
    public static double armKd = 0;
  }

  public static class OperatorConstants {
    public static final int indexerButton = 13;
    public static final int reverseIndexerButton = 4;

    // public static final int groundIntakeButton = 13;
    // public static final int armManualOuttakeButton = 12;

    // public static final int armPickupHeightButton = 5;
    // public static final int armL1HeightButton = 6;
    // public static final int armManualUp = 10;
    // public static final int armManualDown = 9;

    public static final int L4HeightButton = 8;
    public static final int L3HeightButton = 7;
    public static final int L2HeightButton = 6;
    public static final int elevatorDownButton = 5;
    public static final int elevatorManualUp = 10;
    public static final int elevatorManualDown = 9;
    public static final int homeElevatorButon = 2;
    public static final int elevatorOverrideButton = 3;
    public static final int algaeHighPosition = 7;
    public static final int algaeLowPosition = 8;

    public static final int coralInTheWay = 1;

    public static final int outtakeButton = 12;

    public static final int algaeGroundIntakeButton = 5;
    public static final int algaeGroundOutButton = 6;
    public static final int algaeHoldButton = 14;
    public static final int algaeTopButton = 15;

    public static final int algaeModeButton = 16;

    public static final int startingConfigButton = 11;
  }

  public static class PreMatchConstants {
    public static final double prematchDelay = 2.5;
    public static final double prematchDelayBetweenSteps = 0.5;
    public static final Time algaePrematchDelay = Seconds.of(3);
  }
}
