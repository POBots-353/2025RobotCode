package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.util.ExpandedSubsystem;

// TODO:
// Increase Speed of Algae Remover
// Automate it as much as possible.

@Logged(strategy = Strategy.OPT_IN)
public class AlgaeRemover extends ExpandedSubsystem {
  private SparkMax algaeRemoverMotor;
  private TalonFX algaeIntakeMotor;

  private SparkClosedLoopController algaeRemoverPIDController;
  private SparkAbsoluteEncoder algaeRemoverAbsoluteEncoder;
  private RelativeEncoder algaeRemoverEncoder;

  private double positionTolerance = Units.degreesToRadians(3.53);

  public AlgaeRemover() {
    algaeRemoverMotor =
        new SparkMax(AlgaeRemoverConstants.algaeRemoverMotorID, MotorType.kBrushless);
    algaeRemoverAbsoluteEncoder = algaeRemoverMotor.getAbsoluteEncoder();
    algaeRemoverEncoder = algaeRemoverMotor.getEncoder();
    algaeRemoverPIDController = algaeRemoverMotor.getClosedLoopController();
    SparkMaxConfig algaeRemoverConfig = new SparkMaxConfig();

    algaeIntakeMotor = new TalonFX(AlgaeRemoverConstants.algaeIntakeMotorID);

    algaeRemoverConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(AlgaeRemoverConstants.absoluteEncoderConversion)
        .zeroCentered(true);

    algaeRemoverConfig
        .encoder
        .positionConversionFactor(AlgaeRemoverConstants.internalEncoderConversion)
        .velocityConversionFactor(AlgaeRemoverConstants.internalEncoderConversion);
    // .zeroOffset(.113922);

    algaeRemoverConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    algaeRemoverConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeRemoverConstants.currentLimit)
        .secondaryCurrentLimit(AlgaeRemoverConstants.shutOffCurrentLimit);
        
    algaeRemoverConfig.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot0).p(2.5).i(0.0).d(0.0);

    // algaeRemoverConfig
    //     .closedLoop
    //     .maxMotion
    //     .maxVelocity(AlgaeRemoverConstants.maxVelocity)
    //     .maxAcceleration(AlgaeRemoverConstants.maxAcceleration);

    algaeRemoverConfig
        .softLimit
        .forwardSoftLimit(AlgaeRemoverConstants.minPosition.in(Degrees))
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(AlgaeRemoverConstants.maxPosition.in(Degrees))
        .reverseSoftLimitEnabled(true);

    algaeRemoverMotor.configure(
        algaeRemoverConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeIntakeMotor.getConfigurator().apply(AlgaeRemoverConstants.algaeIntakeConfigs);

    algaeRemoverEncoder.setPosition(getAbsolutePosition().in(Radians));
  }

  public Command moveToPosition(Angle targetAngle) {
    return run(() ->
            algaeRemoverPIDController.setReference(
                targetAngle.in(Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0)).until(()-> atSetpoint(targetAngle))
        .withName("Algae Remover move to " + targetAngle.in(Degrees) + " degrees");
  }

  public boolean atSetpoint(Angle targetAngle) {
    return Math.abs(targetAngle.in(Radians) - algaeRemoverEncoder.getPosition()) < positionTolerance;
  }

  public void stopAlgaeRemover() {
    algaeRemoverMotor.set(0);
  }

  public void algaeRemoverUp() {
    algaeRemoverMotor.set(AlgaeRemoverConstants.algaeRemoverSpeed);
  }

  public void algaeRemoverDown() {
    algaeRemoverMotor.set(-AlgaeRemoverConstants.algaeRemoverSpeed);
  }

  @Logged(name = "Absolute Position")
  public Angle getAbsolutePosition() {
    return Radians.of(algaeRemoverAbsoluteEncoder.getPosition());
  }

  @Logged(name = "Position")
  public Angle getPosition() {
    return Radians.of(algaeRemoverEncoder.getPosition());
  }

  public Command intake() {
    return run(() -> algaeIntakeMotor.set(AlgaeRemoverConstants.algaeIntakeSpeed))
        .withName("Run Algae Intake");
  }

  public Command slowIntake() {
    return run(() -> algaeIntakeMotor.set(AlgaeRemoverConstants.slowAlgaeIntakeSpeed))
        .withName("Run Algae Intake");
  }

  public Command outtake() {
    return run(() -> algaeIntakeMotor.set(-AlgaeRemoverConstants.algaeIntakeSpeed))
        .withName("Reverse Algae Intake");
  }

  public Command stop() {
    return runOnce(() -> algaeIntakeMotor.set(0)).withName("Stop Algae Intake");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Remover/Position", getPosition().in(Degrees));
  }

  @Override
  public Command getPrematchCheckCommand() {
    return runOnce(() -> {});
    //   return Commands.sequence(
    //       // Check for hardware errors
    //       Commands.runOnce(
    //           () -> {
    //             REVLibError error = algaeRemoverMotor.getLastError();
    //             if (error != REVLibError.kOk) {
    //               addError("Intake motor error: " + error.name());
    //             } else {
    //               addInfo("Intake motor contains no errors");
    //             }
    //           }),

    //       // Checks Ground Intake Motor
    //       Commands.parallel(
    //           moveToPosition(AlgaeRemoverConstants.intakePosition),
    //           Commands.sequence(
    //               Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
    //               Commands.runOnce(
    //                   () -> {
    //                     if (Math.abs(getPosition()) <= 1e-4) {
    //                       addError("Algae Remover Motor isn't moving");
    //                     } else {
    //                       addInfo("Algae Remover Motor is moving");
    //                       if (Math.abs(AlgaeRemoverConstants.intakePosition.minus(getPosition()))
    // > 0.1) {
    //                         addError("Algae Remover Motor is not at desired position");
    //                         // We just put a fake range for now; we'll update this later on
    //                       } else {
    //                         addInfo("Algae Remover Motor is at the desired position");
    //                       }
    //                     }
    //                   }))),
    //       moveToPosition(AlgaeRemoverConstants.outPosition),
    //       Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
    //       Commands.runOnce(
    //           () -> {
    //             if (Math.abs(getPosition()) > 5) {
    //               addError("Algae Remover Motor isn't fully down");
    //             } else {
    //               addInfo("Algae Rmeover motor is fully down");
    //             }
    //           }));
    // }
  }
}
