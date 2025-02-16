package frc.robot.subsystems;

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
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.util.ExpandedSubsystem;

@Logged(strategy = Strategy.OPT_IN)
public class AlgaeRemover extends ExpandedSubsystem {
  private SparkMax algaeRemoverMotor;
  private SparkClosedLoopController algaeRemoverPIDController;
  private SparkAbsoluteEncoder algaeRemoverAbsoluteEncoder;

  public AlgaeRemover() {
    algaeRemoverMotor =
        new SparkMax(AlgaeRemoverConstants.algaeRemoverMotorID, MotorType.kBrushless);
    algaeRemoverAbsoluteEncoder = algaeRemoverMotor.getAbsoluteEncoder();
    algaeRemoverPIDController = algaeRemoverMotor.getClosedLoopController();
    SparkMaxConfig algaeRemoverConfig = new SparkMaxConfig();

    algaeRemoverConfig.absoluteEncoder.inverted(false).positionConversionFactor(360).zeroOffset(0);

    algaeRemoverConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    algaeRemoverConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    algaeRemoverConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .secondaryCurrentLimit(25);

    algaeRemoverConfig.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot0).p(0.1).i(0).d(0);

    algaeRemoverConfig
        .closedLoop
        .maxMotion
        .maxVelocity(AlgaeRemoverConstants.maxVelocity)
        .maxAcceleration(AlgaeRemoverConstants.maxAcceleration);

    algaeRemoverConfig
        .softLimit
        .forwardSoftLimit(AlgaeRemoverConstants.horizontalPosition)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(AlgaeRemoverConstants.downPosition)
        .reverseSoftLimitEnabled(true);

    algaeRemoverMotor.configure(
        algaeRemoverConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command moveToPosition(double targetAngle) {
    return runOnce(
        () ->
            algaeRemoverPIDController.setReference(
                targetAngle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0));
  }

  public void stopAlgaeRemover() {
    algaeRemoverMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Remover/Position", algaeRemoverAbsoluteEncoder.getPosition());
  }
}
