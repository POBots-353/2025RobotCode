// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class ElTest {
    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Elevator extends SubsystemBase {
  // Creates elevator motor
  private CANSparkMax elevatorMotor = new CANSparkMax(IntakeConstants.elevatorMotorID, MotorType.kBrushless);

  // Creates PID controller for elevatorMotor, which uses PID to control the
  // elevator
  private SparkMaxPIDController elevatorPIDController = elevatorMotor.getPIDController();

  // Creates RelativeEncoder for elevator motor, which gets info about the it
  // (such as speed)
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  // Creates elevatorPiston, which is used in the startingConfiguration
  private DoubleSolenoid elevatorPiston = new DoubleSolenoid(IntakeConstants.pneumaticHubID,
      PneumaticsModuleType.CTREPCM,
      IntakeConstants.elevatorPistonForwardID, IntakeConstants.elevatorPistonReverseID);

  // Creates manipulatorBreak
  private DoubleSolenoid manipulatorBreak = new DoubleSolenoid(IntakeConstants.pneumaticHubID,
      PneumaticsModuleType.CTREPCM,
      IntakeConstants.manipulatorBreakForwardID, IntakeConstants.manipulatorBreakReverseID);

  // Creates the limit switches, which limits the range of motion of the elevator.
  private DigitalInput topLimitSwitch = new DigitalInput(8);
  private DigitalInput bottomLimitSwitch = new DigitalInput(9);

  // Creates the initial variabes for PID, acceleration, speed, etc. for later use
  private int smartMotionSlot = 0;
  private int allowedErr;
  private int minVel;
  private double kP = 5.9721E-05; // 5.15e-4 4.05e-4 1.05e-4
  private double kI = 0;
  private double kD = 0; // 0, 4.05e-4 1.05e-4
  private double kIz = 0;
  private double kFF = 0.0004429; // 0.000206 0.000750 0.0004429
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxVel = 3600; // 5000 2800
  private double maxAcc = 4000; // 2500, 4000

  /** Creates a new ElevatorSubsystem. */
  public Elevator() {

    elevatorMotor.restoreFactoryDefaults();

    // Sets the default position of the elevator position to piston out
    elevatorPiston.set(Value.kForward);

    // Sets the default position of the manipularbreak to piston out
    manipulatorBreak.set(Value.kForward);

    // Initializes the PId for the elevator
    initializePID(elevatorPIDController);

    // Sets the elevator position to the startingConfiguration height
    elevatorEncoder.setPosition(IntakeConstants.startingConfigurationHeight);
  }

  // Initializes PID
  private void initializePID(SparkMaxPIDController p) {
    p.setP(kP); // Sets the Proportional Gain constat of the PIDF contorller
    p.setI(kI); // Sets the Integral Gain constant of the PIDF contoller
    p.setD(kD); // Sets the Derivative Gain constant of the PIDF controller
    p.setIZone(kIz); // Sets the IZone range
    p.setFF(kFF); // Sets the Feed-forward Gain constant of the PIDF controller
    /*
     * Feed-forward is a prediction technique that estimates the output from a PID
     * controller without waiting
     * for the PID algorithm to respond. They are useful for reducing the error
     * fastor or keeps the error smaller
     * rather than making the PID algorithm do it by itself
     */
    p.setOutputRange(kMinOutput, kMaxOutput); /* Sets the minimum and maximum output of the PID controller */
    p.setSmartMotionMaxVelocity(maxVel, smartMotionSlot); /* Sets the maximum velocity of the SmartMotion mode */
    p.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot); /* Sets the minimum velocity of the Smartmotion mode */
    p.setSmartMotionMaxAccel(maxAcc, smartMotionSlot); /* Sets the maximum acceleration of the SmartMotion mode */
    p.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot); /* Sets the allowed error for the motor */
  }

  // The methods below kinda explain themselves
  public void toggleElevatorTilt() {
    elevatorPiston.toggle();
  }

  public void elevatorTiltIn() {
    elevatorPiston.set(Value.kReverse);
  }

  public void elevatorTiltOut() {
    elevatorPiston.set(Value.kForward);
  }

  public void setElevatorPosition(double elevatorPos) {
    elevatorPIDController.setReference(elevatorPos, CANSparkMax.ControlType.kSmartMotion);
  }

  public void elevatorUp() {
    elevatorMotor.set(-IntakeConstants.elevatorSpeed);
  }

  public void elevatorDown() {
    if (bottomSwitchPressed()) {
      elevatorStop();
      return;
    }

    elevatorMotor.set(IntakeConstants.elevatorSpeed);
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  public double getMotorCurrent() {
    return elevatorMotor.getOutputCurrent();
  }

  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  public void zeroElevatorPosition() {
    elevatorEncoder.setPosition(0);
  }

  public boolean topSwitchPressed() {
    return !topLimitSwitch.get();
  }

  public boolean bottomSwitchPressed() {
    return !bottomLimitSwitch.get();
  }

  public void toggleOnManipulatorBreak() {
    manipulatorBreak.set(Value.kForward);
  }

  public void toggleOffManipulatorBreak() {
    manipulatorBreak.set(Value.kReverse);
  }

  public DoubleSolenoid.Value getPistonState() {
    return elevatorPiston.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition()); /*
                                                                                   * Posts the
                                                                                   * elevator position to SmartDashboard
                                                                                   */
    SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity());

    // Posts if the limit switches are triggered or not
    SmartDashboard.putBoolean("Top Switch", !topLimitSwitch.get());
    SmartDashboard.putBoolean("Bottom Switch", !bottomLimitSwitch.get());

    // Posts the elevator motor temperature
    SmartDashboard.putNumber("Elevator Motor Temperature", elevatorMotor.getMotorTemperature());

    // Posts the elevator motor current (voltage)
    SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());

    if (!bottomLimitSwitch.get()) {
      elevatorEncoder.setPosition(0);
    } // if the bottom limit switch is triggered, zero the elevator

    // SmartDashboard.putNumber("Pressure", pneumaticHub.getPressure(0));
  }
}
}
