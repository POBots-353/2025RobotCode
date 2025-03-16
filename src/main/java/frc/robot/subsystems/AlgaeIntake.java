package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeRemoverConstants;
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
}
