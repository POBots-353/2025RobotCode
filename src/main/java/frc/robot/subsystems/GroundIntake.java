// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private SparkMax groundIntakeMotor;

  private LaserCan intakeLaser;

  public GroundIntake() {
    groundIntakeMotor = new SparkMax(IntakeConstants.groundIntakeMotorID, MotorType.kBrushless);
    intakeLaser = new LaserCan(0);

    SparkMaxConfig groundIntakeConfig = new SparkMaxConfig();

    groundIntakeConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.groundIntakeCurrentLimit)
        .secondaryCurrentLimit(IntakeConstants.groundIntakeShutOffLimit);

    groundIntakeMotor.configure(
        groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public boolean intakeLaserBroken() {
    LaserCan.Measurement measurement = intakeLaser.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      // System.out.println("The target is " + measurement.distance_mm + "mm away!");
      // if (measurement.distance_mm < 500) {
      //   return true;
      // } else {
      //   return false;
      // }
      return true;
    } else {
      return false;
    }
  }

  public void stopGroundIntake() {
    groundIntakeMotor.set(0);
  }

  public void groundIntake() {
    if (!intakeLaserBroken()) {
      groundIntakeMotor.set(IntakeConstants.groundIntakeMotorSpeed);
    } else {
      stopGroundIntake();
    }
  }

  public Command runIntake() {
    return run(this::groundIntake);
  }

  public Command stop() {
    return runOnce(this::stopGroundIntake);
  }

  public void feedToIndexer() {
    groundIntakeMotor.set(IntakeConstants.groundIntakeMotorSpeed);
  }

  public void manualOutake() {
    groundIntakeMotor.set(IntakeConstants.outakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
