// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorMainMotor;

  private TalonFX elevatorFollowerMotor;
  private Follower follower;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private DigitalInput buttonSwitch = new DigitalInput(ElevatorConstants.buttonSwitchID);
  private boolean isZeroed = true; // false
  private Alert elevatorAlert;

  public Elevator() {
    elevatorMainMotor = new TalonFX(ElevatorConstants.elevatorMainMotorID, "Cannie");
    elevatorFollowerMotor = new TalonFX(ElevatorConstants.elevatorFollowerMotorID, "Cannie");
    follower = new Follower(ElevatorConstants.elevatorMainMotorID, false);
    // elevatorMainMotor.setPosition(Rotations.of(0));

    // currentPosition.setUpdateFrequency(50);

    // elevatorFollowerMotor.setControl(follower);

    elevatorMainMotor.getConfigurator().apply(ElevatorConstants.elevatorConfigs);

    elevatorAlert = new Alert("Elevator is not Zeroed!", AlertType.kWarning);
  }

  public boolean buttonPressed() {
    return !buttonSwitch.get();
  }

  public Command homeElevator() {
    return run(() -> setSpeed(-0.2))
        .until(this::buttonPressed)
        .unless(this::buttonPressed)
        .finallyDo(this::stopElevator);
  }

  public void stopElevator() {
    elevatorMainMotor.set(0);
  }

  public void setSpeed(double speed) {
    if (elevatorMainMotor.getPosition().getValueAsDouble() < 50) {
      elevatorMainMotor.set(ElevatorConstants.bottomSpeed);
    } else {
      elevatorMainMotor.set(speed);
    }
  }

  public void printPosition() {
    SmartDashboard.putNumber(
        "Elevator Position", elevatorMainMotor.getPosition().getValueAsDouble());
  }

  // public void safelySetControl(double height) {
  //   if (elevatorMainMotor.getPosition().getValueAsDouble() < 50) {
  //     elevatorMainMotor.set(ElevatorConstants.bottomSpeed);
  //   } else {
  //     elevatorMainMotor.setControl(motionMagicRequest.withPosition(height));
  //   }
  // }

  public Command moveToPosition(double height) {
    System.out.println("OAIUBDAIFBAIFJBASIUFBAISUFASIFUB"); 
    return run(() -> elevatorMainMotor.setControl(motionMagicRequest.withPosition(height)))
        .onlyIf(() -> isZeroed)
        .until(this::buttonPressed);
    // return run(() -> safelySetControl(height)).onlyIf(() ->isZeroed).until(this::buttonPressed);
  }

  public Command downPosition() {
    return moveToPosition(0.0);
  }

  private final SysIdRoutine elevatorSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdElevator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> elevatorMainMotor.setControl(voltageRequest.withOutput(volts.in(Volts))),
              null,
              this));

  public Command sysIdQuasistaticElevator(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamicElevator(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    printPosition();

    if (!isZeroed && buttonPressed()) {
      elevatorMainMotor.setPosition(0, .001);
      isZeroed = true;
    }

    elevatorAlert.set(!isZeroed);
  }
}
