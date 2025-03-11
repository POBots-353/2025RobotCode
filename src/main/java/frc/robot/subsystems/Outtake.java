// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.util.ExpandedSubsystem;

@Logged(strategy = Strategy.OPT_IN)
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

    // outtakeConfig
    //     .signals
    //     .absoluteEncoderPositionAlwaysOn(false)
    //     .absoluteEncoderVelocityAlwaysOn(false)
    //     .primaryEncoderPositionAlwaysOn(false)
    //     .externalOrAltEncoderPositionAlwaysOn(false)
    //     .externalOrAltEncoderVelocityAlwaysOn(false);

    outtakemotor.configure(
        outtakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command fastOuttake() {
    return run(() -> outtakemotor.set(OuttakeConstants.fastOuttakeSpeed));
  }

  public Command slowOuttake() {
    return run(() -> outtakemotor.set(OuttakeConstants.slowOuttakeSpeed));
  }

  public Command reverseOuttake() {
    return run(() -> outtakemotor.set(-OuttakeConstants.fastOuttakeSpeed));
  }

  public Command outtakeUntilBeamBreak() {
    return slowOuttake()
        .unless(this::outtakeLaserBroken)
        .until(this::outtakeLaserBroken)
        .finallyDo(this::stop); // ()-> Commands.waitSeconds(.05).andThen(this::stop));
  }

  public Command autoOuttake() {
    return fastOuttake().until(() -> !outtakeLaserBroken()).finallyDo(this::stop);
  }

  public void stop() {
    outtakemotor.set(0.00);
  }

  public Command stopOuttakeMotor() {
    return runOnce(this::stop);
  }

  public boolean outtakeLaserBroken() {
    LaserCan.Measurement measurement = outtakeLaser.getMeasurement();
    if (measurement != null
        && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && measurement.distance_mm < 75) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {}

  @Override
  public Command getPrematchCheckCommand() {
    return Commands.sequence(
        // Check for hardware errors
        Commands.runOnce(
            () -> {
              REVLibError error = outtakemotor.getLastError();
              if (error != REVLibError.kOk) {
                addError("Intake motor error: " + error.name());
              } else {
                addInfo("Intake motor contains no errors");
              }
            }),
        // Checks Indexer Motor
        Commands.parallel(
            fastOuttake(),
            Commands.sequence(
                Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(outtakemotor.get()) <= 1e-4) {
                        addError("Outtake Motor is not moving");
                      } else {
                        addInfo("Outtake Motor is moving");
                        if (Math.abs(OuttakeConstants.fastOuttakeSpeed - outtakemotor.get())
                            > 0.1) {
                          addError("Outtake Motor is not at fast velocity");
                          // We just put a fake range for now; we'll update this later on
                        } else {
                          addInfo("Outtake Motor is at the fast velocity");
                        }
                      }
                    }))),
        Commands.parallel(
            slowOuttake(),
            Commands.sequence(
                Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(outtakemotor.get()) <= 1e-4) {
                        addError("Outtake Motor is not moving");
                      } else {
                        addInfo("Outtake Motor is moving");
                        if (Math.abs(OuttakeConstants.slowOuttakeSpeed - outtakemotor.get())
                            > 0.1) {
                          addError("Outtake Motor is not at slow velocity");
                          // We just put a fake range for now; we'll update this later on
                        } else {
                          addInfo("Outtake Motor is at the slow velocity");
                        }
                      }
                    }))),
        Commands.runOnce(() -> stop()),
        Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(outtakemotor.get()) > 0.1) {
                addError("Outtake Motor isn't stopping");
              } else {
                addInfo("Outtake Motor did stop");
              }
            }));
  }
}
