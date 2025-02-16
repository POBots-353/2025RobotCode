// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.ExpandedSubsystem;

@Logged(strategy = Strategy.OPT_IN)
public class Arm extends ExpandedSubsystem {
  private SparkMax armMotor;
  private SparkClosedLoopController armPIDController;
  private SparkAbsoluteEncoder armAbsoluteEncoder;

  public Arm() {
    armMotor = new SparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();
    armPIDController = armMotor.getClosedLoopController();
    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig.absoluteEncoder.inverted(false).positionConversionFactor(360).zeroOffset(0);
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    armConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .secondaryCurrentLimit(25);

    armConfig.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot0).p(0).i(0).d(0);

    armConfig
        .closedLoop
        .maxMotion
        .maxVelocity(ArmConstants.armMaxVelocity)
        .maxAcceleration(ArmConstants.armMaxAcceleration);

    armConfig
        .softLimit
        .forwardSoftLimit(100)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-100)
        .reverseSoftLimitEnabled(true);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command moveToPosition(double targetAngle) {
    return runOnce(
        () ->
            armPIDController.setReference(
                targetAngle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0));
  }

  public Command manualUp(double speed) {
    return run(() -> armMotor.set(speed));
  }

  public Command manualDown(double speed) {
    return run(() -> armMotor.set(speed));
  }



  public void stopArm() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Rotations", armAbsoluteEncoder.getPosition());
  }
}
