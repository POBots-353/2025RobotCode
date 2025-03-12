package frc.robot.subsystems;

import com.revrobotics.REVLibError;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.util.ExpandedSubsystem;

// TODO:
// Increase Speed of Algae Remover
// Automate it as much as possible.

@Logged(strategy = Strategy.OPT_IN)
public class AlgaeRemover extends ExpandedSubsystem {
  private SparkMax algaeRemoverMotor;
  private SparkMax algaeIntakeMotor;

  private SparkClosedLoopController algaeRemoverPIDController;
  private SparkAbsoluteEncoder algaeRemoverAbsoluteEncoder;

  public AlgaeRemover() {
    algaeRemoverMotor =
        new SparkMax(AlgaeRemoverConstants.algaeRemoverMotorID, MotorType.kBrushless);
    algaeRemoverAbsoluteEncoder = algaeRemoverMotor.getAbsoluteEncoder();
    algaeRemoverPIDController = algaeRemoverMotor.getClosedLoopController();
    SparkMaxConfig algaeRemoverConfig = new SparkMaxConfig();

    algaeIntakeMotor = new SparkMax(AlgaeRemoverConstants.algaeIntakeMotorID, MotorType.kBrushless);

    algaeRemoverConfig.absoluteEncoder.inverted(false);
    // .positionConversionFactor(360 * (18.0 / 32.0))
    // .zeroOffset(.113922); // .113922

    algaeRemoverConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    algaeRemoverConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeRemoverConstants.currentLimit)
        .secondaryCurrentLimit(AlgaeRemoverConstants.shutOffCurrentLimit);
    algaeRemoverConfig.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot0).p(30).i(0.0).d(0.0);

    algaeRemoverConfig
        .closedLoop
        .maxMotion
        .maxVelocity(AlgaeRemoverConstants.maxVelocity)
        .maxAcceleration(AlgaeRemoverConstants.maxAcceleration);

    // algaeRemoverConfig
    //     .softLimit
    //     .forwardSoftLimit(AlgaeRemoverConstants.HighestPosition)
    //     .forwardSoftLimitEnabled(true)
    //     .reverseSoftLimit(AlgaeRemoverConstants.LowestPosition)
    //     .reverseSoftLimitEnabled(true);

    SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();

    algaeIntakeConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeRemoverConstants.currentLimit)
        .secondaryCurrentLimit(AlgaeRemoverConstants.shutOffCurrentLimit);

    algaeRemoverMotor.configure(
        algaeRemoverConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeIntakeMotor.configure(
        algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command moveToPosition(double targetAngle) {
    return run(() ->
            algaeRemoverPIDController.setReference(
                targetAngle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0))
        .withName("Algae Remover move to " + targetAngle);
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

  public double getPosition() {
    return algaeRemoverAbsoluteEncoder.getPosition(); // in degrees
  }

  public Command intake() {
    return run(() -> algaeIntakeMotor.set(AlgaeRemoverConstants.algaeIntakeSpeed))
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
    SmartDashboard.putNumber("Algae Remover/Position", getPosition());
  }

  @Override
  public Command getPrematchCheckCommand() {
    return Commands.sequence(
        // Check for hardware errors
        Commands.runOnce(
            () -> {
              REVLibError error = algaeRemoverMotor.getLastError();
              if (error != REVLibError.kOk) {
                addError("Intake motor error: " + error.name());
              } else {
                addInfo("Intake motor contains no errors");
              }
            }),

        // Checks Ground Intake Motor
        Commands.parallel(
            moveToPosition(AlgaeRemoverConstants.intakePosition),
            Commands.sequence(
                Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(getPosition()) <= 1e-4) {
                        addError("Algae Remover Motor isn't moving");
                      } else {
                        addInfo("Algae Remover Motor is moving");
                        if (Math.abs(AlgaeRemoverConstants.intakePosition - getPosition()) > 0.1) {
                          addError("Algae Remover Motor is not at desired position");
                          // We just put a fake range for now; we'll update this later on
                        } else {
                          addInfo("Algae Remover Motor is at the desired position");
                        }
                      }
                    }))),
        moveToPosition(AlgaeRemoverConstants.outPosition),
        Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(getPosition()) > 5) {
                addError("Algae Remover Motor isn't fully down");
              } else {
                addInfo("Algae Rmeover motor is fully down");
              }
            }));
  }
}
