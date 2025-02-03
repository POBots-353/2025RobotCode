// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.ExpandedSubsystem;

public class AlgaeIntake extends ExpandedSubsystem {
  /** Creates a new AlgaeIntake. */
  private SparkMax algaeIntakeMotor =
      new SparkMax(AlgaeIntakeConstants.algaeIntakeMotorID, MotorType.kBrushless);

  private RelativeEncoder AlgaeIntake = algaeIntakeMotor.getEncoder();

  public AlgaeIntake() {
    DataLogManager.log("Configuring Intake");
    SparkMaxConfig algaeConfig = new SparkMaxConfig();
    algaeConfig
        .inverted(true)
        .smartCurrentLimit(IntakeConstants.intakeCurrentLimit)
        .secondaryCurrentLimit(IntakeConstants.algaeIntakeShutoffCurrentLimit)
        .idleMode(IdleMode.kBrake);
  }

  public void startAlgaeIntake() {
    algaeIntakeMotor.set(AlgaeIntakeConstants.algaeIntakeSpeed);
  }

  public void stopAlgaeIntake() {
    algaeIntakeMotor.set(0);
  }

  //   @Override
  //   public Command getPrematchCheckCommand(
  //     VirtualXboxController controller, virtualJoystick joystick) {
  //       return Commands.sequence(
  //         Commands.runOnce(
  //           () -> {
  //             REVLibError error = algaeIntakeMotor.getLastError();
  //             if (error != REVLibError.kOk) {
  //               addError("Intake motor error: " + error.name());
  //             } else {
  //               addInfo("Intake motor contains no errors");
  //             }
  //           });
  //           Commands.runOnce(
  //             () -> {
  //               joystick.setButton(OperatorConstants.intakeNoteButton, true);
  //             }),
  //             Commands.waitSeconds(prematchDelay),
  //             Commands.runOnce(
  //               () -> {
  //                 if (Math.abs(intakeEncoder.getVelocity()) <= 1e-4) {
  //                   addError("Intake Motor is not moving");
  //                 } else {
  //                   addInfo("Intake Motor is moving");
  //                 }
  //                 joystick.clearVirtualButtons();
  //               }),
  //               Commands.runOnce(
  //                 () -> {
  //                   joystick.clearVirtualButtons();
  //                 }));
  //     }
  // }

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Temperature", algaeIntakeMotor.getMotorTemperature());
  }

  // public Command getPrematchCheckCommand(
  //     CommandXboxController controller, CommandJoystick joystick) {
  //   return Commands.sequence(
  //       // Check for hardware motors
  //       Commands.runOnce(
  //           () -> {
  //             REVLibError error = algaeIntakeMotor.getLastError();
  //             if (error != REVLibError.kOk) {
  //               addError("Climber motor error: " + error.name());
  //             } else {
  //               addInfo("Climber motor contains no errors");
  //             }
  //           }),
  //       // Checks climber motor
  //       Commands.runOnce(
  //           () -> {
  //             joystick.setButton(OperatorConstants.climberButton, true);
  //           }),
  //       Commands.waitSeconds(prematchDelay),
  //       Commands.runOnce(
  //           () -> {
  //             if (getVelocity() < 10) {
  //               addError("Climber motor isn't working");
  //             } else {
  //               addInfo("Climber motor is moving");
  //             }
  //             joystick.clearVirtualButtons();
  //           }));
  // }
}
