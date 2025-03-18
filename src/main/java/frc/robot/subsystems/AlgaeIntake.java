package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.Constants.PreMatchConstants;
import frc.robot.util.ExpandedSubsystem;

// TODO:
// Increase Speed of Algae Remover
// Automate it as much as possible.

@Logged(strategy = Strategy.OPT_IN)
public class AlgaeIntake extends ExpandedSubsystem {
  private TalonFX algaeIntakeMotor;

  public AlgaeIntake() {

    algaeIntakeMotor = new TalonFX(AlgaeRemoverConstants.algaeIntakeMotorID);
    algaeIntakeMotor.getConfigurator().apply(AlgaeRemoverConstants.algaeIntakeConfigs);
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
  public void periodic() {}

  @Override
  public Command getPrematchCheckCommand() {
    return Commands.sequence(
        Commands.parallel(
            intake(),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelayBetweenSteps),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(algaeIntakeMotor.get()) <= 1e-4) {
                        addError("Algae Intake Motor is not intaking");
                      } else {
                        addInfo("Algae Intake Motor is intaking");
                        if (algaeIntakeMotor.get() < 0) {
                          addError("Algae Intake Motor is intaking in the wrong direction");
                          // We just put a fake range for now; we'll update this later on
                        } else {
                          addInfo("Algae Intake Motor is at the velocity for intake");
                        }
                      }
                    }))),
        Commands.parallel(
            slowIntake(),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelayBetweenSteps),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(algaeIntakeMotor.get()) <= 1e-4) {
                        addError("Algae Intake Motor is not slow intaking");
                      } else {
                        addInfo("Algae Intake Motor is moving");
                        if (algaeIntakeMotor.get() < 0) {
                          addError("Algae Intake Motor is intaking in the wrong direction");
                          // We just put a fake range for now; we'll update this later on
                        } else {
                          addInfo("Algae Intake Motor is at the velocity for intake");
                        }
                      }
                    }))),
        Commands.parallel(
            outtake(),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelayBetweenSteps),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(algaeIntakeMotor.get()) <= 1e-4) {
                        addError("Algae Intake Motor is not outtaking");
                      } else {
                        addInfo("Algae Intake Motor is moving");
                        if (algaeIntakeMotor.get() > 0) {
                          addError("Algae Intake Motor is intaking in the wrong direction");
                          // We just put a fake range for now; we'll update this later on
                        } else {
                          addInfo("Algae Intake Motor is at the velocity for outtake");
                        }
                      }
                    }))),
        Commands.parallel(
            stop(),
            Commands.sequence(
                Commands.waitTime(PreMatchConstants.prematchDelayBetweenSteps),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(algaeIntakeMotor.get()) > 0.1) {
                        addError("Algae Intake Motor is moving");
                      } else {
                        addInfo("Algae Intake Stopped");
                      }
                    }))));
  }
}
