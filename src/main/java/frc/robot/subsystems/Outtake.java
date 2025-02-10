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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.util.ExpandedSubsystem;

@Logged
public class Outtake extends ExpandedSubsystem {

  private SparkMax outtakemotor;
  private LaserCan outtakeLaser;

  public Outtake() {
    outtakemotor = new SparkMax(OuttakeConstants.outtakeMotorID, MotorType.kBrushless);
    outtakeLaser = new LaserCan(OuttakeConstants.outtakeLaserCanID);

    SparkMaxConfig outtakeConfig = new SparkMaxConfig();

    outtakeConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(OuttakeConstants.outtakeCurrentLimit)
        .secondaryCurrentLimit(OuttakeConstants.outtakeShutOffLimit);

    outtakemotor.configure(
        outtakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void moveOuttake() {
    if (!outtakeLaserBroken()) {
      outtakemotor.set(OuttakeConstants.outtakeSpeed);
    }
    outtakemotor.set(0.0);
  }

  public Command runOuttake() {
    return run(this::moveOuttake);
  }

  public Command outtakeUntilBeamBreak() {
    return run(this::moveOuttake)
        .unless(this::outtakeLaserBroken)
        .until(this::outtakeLaserBroken)
        .finallyDo(this::stopOuttakeMotor);
  }

  public Command autoOuttake() {
    return run(this::moveOuttake).until(() -> !outtakeLaserBroken());
  }

  public void stop() {
    outtakemotor.set(0.00);
  }

  public Command stopOuttakeMotor() {
    return runOnce(this::stop);
  }

  public Boolean outtakeLaserBroken() {
    LaserCan.Measurement measurement = outtakeLaser.getMeasurement();
    if (measurement != null
        && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && measurement.distance_mm < 100) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {}
}
