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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.util.ExpandedSubsystem;

public class Outtake extends ExpandedSubsystem {

  private SparkMax outtakemotor;
  private LaserCan outtakeLaser;

  public Outtake() {
    outtakemotor = new SparkMax(OuttakeConstants.outtakeMotorID, MotorType.kBrushless);
    outtakeLaser = new LaserCan(OuttakeConstants.outtakeLaserCanID);

    SparkMaxConfig outtakeConfig = new SparkMaxConfig();

    outtakeConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(OuttakeConstants.outtakeCurrentLimit)
        .secondaryCurrentLimit(OuttakeConstants.outtakeShutOffLimit);

    outtakemotor.configure(
        outtakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command runOuttake() {
    return Commands.run(() -> outtakemotor.set(OuttakeConstants.outtakeSpeed));
  }

  public void stop() {
    outtakemotor.set(0.00);
  }

  public Command stopOuttakeMotor() {
    return runOnce(this::stop);
  }

  public boolean outtakeLaserBroken() {
    LaserCan.Measurement measurement = outtakeLaser.getMeasurement();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
