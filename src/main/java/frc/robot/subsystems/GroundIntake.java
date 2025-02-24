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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.ExpandedSubsystem;

@Logged(strategy = Strategy.OPT_IN)
public class GroundIntake extends ExpandedSubsystem {
  private SparkMax groundIntakeMotor;

  private final double prematchDelay = 2.5;

  /** Creates a new GroundIntake. */
  public GroundIntake() {
    groundIntakeMotor = new SparkMax(IntakeConstants.groundIntakeMotorID, MotorType.kBrushless);

    SparkMaxConfig groundIntakeConfig = new SparkMaxConfig();

    groundIntakeConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.groundIntakeCurrentLimit)
        .secondaryCurrentLimit(IntakeConstants.groundIntakeShutOffLimit);

    groundIntakeMotor.configure(
        groundIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stopGroundIntake() {
    groundIntakeMotor.set(0);
  }

  public void groundIntake() {
    groundIntakeMotor.set(IntakeConstants.groundIntakeMotorSpeed);
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

  public void manualOuttake() {
    groundIntakeMotor.set(IntakeConstants.outtakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public Command getPrematchCheckCommand() {
    return Commands.sequence(
        // Check for hardware errors
        Commands.runOnce(
            () -> {
              REVLibError error = groundIntakeMotor.getLastError();
              if (error != REVLibError.kOk) {
                addError("Ground Intake motor error: " + error.name());
              } else {
                addInfo("Ground Intake motor contains no errors");
              }
            }),
        Commands.parallel(
            Commands.runOnce(
                () -> {
                  groundIntake();
                }),
            Commands.sequence(
                Commands.waitSeconds(prematchDelay),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(groundIntakeMotor.get()) <= 1e-4) {
                        addError("Ground Intake Motor is not moving");
                      } else {
                        addInfo("Ground Intake Motor is moving");
                        if ((Math.abs(IntakeConstants.groundIntakeMotorSpeed)
                                - groundIntakeMotor.get())
                            > 0.1) {
                          addError("Ground Intake Motor is not at desired velocity");
                          // We just put a fake range for now; we'll update this later on
                        } else {
                          addInfo("Ground Intake Motor is at the desired velocity");
                        }
                      }
                    }))),
        // Checks Ground Intake Motor
        stop(),
        Commands.waitSeconds(prematchDelay),
        Commands.runOnce(
            () -> {
              if (groundIntakeMotor.get() > 0.1) {
                addError("Ground Intake Motor is not stopping");
              } else {
                addInfo("Ground Intake Motor Stopped");
              }
            }));
  }
}
