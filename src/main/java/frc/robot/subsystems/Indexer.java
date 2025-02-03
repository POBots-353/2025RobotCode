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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.ExpandedSubsystem;
import java.util.ArrayList;
import java.util.List;

public class Indexer extends ExpandedSubsystem {
  /** Creates a new Indexer. */
  private SparkMax indexerMotor;

  private final double prematchDelay = 2.5;

  public List<Alert> indexerPrematchAlerts = new ArrayList<Alert>();

  public Indexer() {
    indexerMotor = new SparkMax(IntakeConstants.indexerMotorID, MotorType.kBrushless);

    SparkMaxConfig indexerConfig = new SparkMaxConfig();

    indexerConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.indexerCurrentLimit)
        .secondaryCurrentLimit(IntakeConstants.indexerShutOffLimit);

    indexerMotor.configure(
        indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command runIndexer() {
    return run(this::index);
  }

  public Command stop() {
    return runOnce(this::stopIndexer);
  }

  public void index() {
    indexerMotor.set(IntakeConstants.indexerMotorSpeed);
  }

  public void stopIndexer() {
    indexerMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Indexer Speed", indexerMotor.get());
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
        Commands.runOnce(
            () -> {
              index();
            }),
        Commands.waitSeconds(prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(indexerMotor.get()) <= 1e-4) {
                if (indexerMotor.get() < IntakeConstants.indexerMotorSpeed - 0.1
                    || indexerMotor.get() > IntakeConstants.indexerMotorSpeed + 0.1) {
                  addError("Indexer Motor is not at desired velocity");
                  // We just put a fake range for now; we'll update this later on
                }
                addError("Indexer Motor is not moving");
              } else {
                addInfo("Indexer Motor is moving");
              }
            }));
  }
}
