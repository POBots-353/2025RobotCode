// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.util.ExpandedSubsystem;
import java.util.ArrayList;
import java.util.List;

@Logged(strategy = Strategy.OPT_IN)
public class Indexer extends ExpandedSubsystem {
  private SparkMax indexerMotor;
  private SparkMax followerMotor;

  public List<Alert> indexerPrematchAlerts = new ArrayList<Alert>();

  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotor = new SparkMax(IntakeConstants.indexerMotorID, MotorType.kBrushless);
    followerMotor = new SparkMax(IntakeConstants.indexerFollowerMotorID, MotorType.kBrushless);

    SparkMaxConfig indexerConfig = new SparkMaxConfig();

    indexerConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.indexerCurrentLimit)
        .secondaryCurrentLimit(IntakeConstants.indexerShutOffLimit);

    SparkMaxConfig followerConfig = new SparkMaxConfig();

    followerConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.indexerCurrentLimit)
        .secondaryCurrentLimit(IntakeConstants.indexerShutOffLimit);

    // indexerConfig
    //     .signals
    //     .absoluteEncoderPositionAlwaysOn(false)
    //     .absoluteEncoderVelocityAlwaysOn(false)
    //     .primaryEncoderPositionAlwaysOn(false)
    //     .externalOrAltEncoderPositionAlwaysOn(false)
    //     .externalOrAltEncoderVelocityAlwaysOn(false)
    //     .analogVelocityAlwaysOn(false)
    //     .analogPositionAlwaysOn(false)
    //     .iAccumulationAlwaysOn(false);

    indexerMotor.configure(
        indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getSpeed() {
    return indexerMotor.get();
  }

  public Command runIndexer() {
    return run(this::index).withName("Run Indexer");
  }

  public Command reverseIndexer() {
    return run(this::reverse).withName("Reverse Indexer");
  }

  public Command stop() {
    return runOnce(this::stopIndexer).withName("Stop Indexer");
  }

  public void reverse() {
    indexerMotor.set(-IntakeConstants.indexerMotorSpeed);
    followerMotor.set(-IntakeConstants.indexerMotorSpeed);
  }

  public void index() {
    indexerMotor.set(IntakeConstants.indexerMotorSpeed);
    followerMotor.set(IntakeConstants.indexerMotorSpeed);
  }

  public void stopIndexer() {
    indexerMotor.set(0);
    followerMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Indexer/Speed", indexerMotor.get());
  }

  @Override
  public Command getPrematchCheckCommand() {
    return Commands.sequence(
        // Check for hardware errors
        Commands.runOnce(
            () -> {
              REVLibError error = indexerMotor.getLastError();
              if (error != REVLibError.kOk) {
                addError("Intake motor error: " + error.name());
              } else {
                addInfo("Intake motor contains no errors");
              }
            }),
        // Checks Indexer Motor
        Commands.parallel(
            Commands.runOnce(
                () -> {
                  index();
                }),
            Commands.sequence(
                Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(indexerMotor.get()) <= 1e-4) {
                        addError("Indexer Motor is not moving");
                      } else {
                        addInfo("Indexer Motor is moving");
                        if (Math.abs(IntakeConstants.indexerMotorSpeed - indexerMotor.get())
                            > 0.1) {
                          addError("Indexer Motor is not at desired velocity");
                          // We just put a fake range for now; we'll update this later on
                        } else {
                          addInfo("Indexer Motor is at the desired velocity");
                        }
                      }
                    }))),
        Commands.runOnce(() -> stopIndexer()),
        Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(indexerMotor.get()) > 0.1) {
                addError("Indexer Motor isn't stopping");
              } else {
                addInfo("Indexer Motor did stop");
              }
            }));
  }
}
