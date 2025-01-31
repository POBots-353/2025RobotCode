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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private SparkMax armMotor;
  private SparkClosedLoopController armPIDController;
  private SparkAbsoluteEncoder armAbsoluteEncoder;

  /** Creates a new CoralArmIntake. */
  public Arm() {
    armMotor = new SparkMax(47, MotorType.kBrushless);
    // armAbsoluteEncoder = armMotor.getAbsoluteEncoder();
    armPIDController = armMotor.getClosedLoopController();
    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(10)
        .secondaryCurrentLimit(15);
    // armConfig.absoluteEncoder.positionConversionFactor(0).velocityConversionFactor(0);
    armConfig
        .closedLoop
        // .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        .p(0)
        .i(0)
        .d(0);
    armConfig.closedLoop.maxMotion.maxVelocity(0).maxAcceleration(0);
    armConfig
        .softLimit
        .forwardSoftLimit(50)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-50)
        .reverseSoftLimitEnabled(true);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void armBottom() {
    armPIDController.setReference(0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  public void armUp() {
    // armPIDController.setReference(10, ControlType.kMAXMotionPositionControl,
    // ClosedLoopSlot.kSlot0);
    armMotor.set(1);
  }

  public void stopArm() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Arm Rotations", armAbsoluteEncoder.getPosition());
  }
}
