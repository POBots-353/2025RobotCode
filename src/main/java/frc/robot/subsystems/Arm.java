// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.util.ExpandedSubsystem;

@Logged(strategy = Strategy.OPT_IN)
public class Arm extends ExpandedSubsystem {
  private SparkMax armMotor;
  private SparkClosedLoopController armPIDController;
  private SparkAbsoluteEncoder armAbsoluteEncoder;
  
  private TrapezoidProfile profile = new TrapezoidProfile(ArmConstants.constraints);
  private TrapezoidProfile.State previousSetpoint = new TrapezoidProfile.State();

  private ArmFeedforward armFeedforward =
      new ArmFeedforward(
          ArmConstants.armKg, ArmConstants.armKs, ArmConstants.armKv, ArmConstants.armKa);

  private PIDController pidController =
  new PIDController(ArmConstants.armKp, ArmConstants.armKi, ArmConstants.armKd);

  private Debouncer setpointDebouncer = new Debouncer(.353);



  public Arm() {
    armMotor = new SparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();
    armPIDController = armMotor.getClosedLoopController();
    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig.absoluteEncoder.inverted(false).positionConversionFactor(360).zeroOffset(0);
    armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    armConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .secondaryCurrentLimit(25);

    armConfig.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot0).p(0).i(0).d(0);

    armConfig
        .closedLoop
        .maxMotion
        .maxVelocity(ArmConstants.armMaxVelocity)
        .maxAcceleration(ArmConstants.armMaxAcceleration);

    armConfig
        .softLimit
        .forwardSoftLimit(100)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-100)
        .reverseSoftLimitEnabled(true);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setDesiredPosition(Rotation2d position){
    TrapezoidProfile.State currentState = previousSetpoint;
    TrapezoidProfile.State targetState = new TrapezoidProfile.State(position.getRadians(), 0.0);

    TrapezoidProfile.State setpoint = profile.calculate(.02, currentState, targetState);

    double positionError = Math.abs(setpoint.position - getPosition().getRadians());

    if (positionError > ArmConstants.replanningError) {
      setpoint = profile.calculate(0.020, getCurrentState(), targetState);//Position error too high
    } else if (profile.isFinished(0.0) && positionError > Units.degreesToRadians(7.5)) {
      setpoint = profile.calculate(0.020, getCurrentState(), targetState);//current profile is finished but too far from setpoint
    }

    setMotionProfileState(setpoint);
  }

  private void setMotionProfileState(TrapezoidProfile.State state) {
    double feedforward = armFeedforward.calculate(state.position, state.velocity, (state.velocity - previousSetpoint.velocity) / 0.020);

    previousSetpoint = state;

    double pidOutput = pidController.calculate(getPosition().getRadians(), state.position);
    pidOutput = MathUtil.clamp(pidOutput, -1.0, 1.0);

    armMotor.setVoltage(pidOutput * RobotController.getBatteryVoltage() + feedforward);
  }

  public void setSpeed(double speed) {
    armMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }


  public Command moveToPosition(Rotation2d position) {
    return new FunctionalCommand(()-> previousSetpoint = getCurrentState(),
    ()-> setDesiredPosition(position),
    (interrupted) -> setSpeed(0),
    () -> {
      double positionError = getPosition().getRadians() - position.getRadians();
      return setpointDebouncer.calculate(
          Math.abs(positionError) <= ArmConstants.angleTolerance);
    }, 
    this);

  }

  public Command manualUp(double speed) {
    return run(() -> armMotor.set(speed));
  }

  public Command manualDown(double speed) {
    return run(() -> armMotor.set(speed));
  }

  // public Command armBottom() {
  //   return runOnce(
  //       () ->
  //           armPIDController.setReference(
  //               -100, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0));
  // }

  // public Command armTop() {
  //   return runOnce(
  //       () ->
  //           armPIDController.setReference(
  //               100, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0));
  // }

  public void stopArm() {
    armMotor.set(0);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(armAbsoluteEncoder.getPosition());
  }

  public TrapezoidProfile.State getCurrentState(){
    return new TrapezoidProfile.State(getPosition().getRadians(), armAbsoluteEncoder.getVelocity());
  }

  public Rotation2d getPosition(){
    return getAngle().minus(Rotation2d.fromDegrees(0));//put offset where 0 is
  }

  public double getPrematchPosition() {
    double rotations = armAbsoluteEncoder.getPosition();
    double degrees = rotations * 360;
    return degrees;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Rotations", armAbsoluteEncoder.getPosition());
  }

  @Override
  public Command getPrematchCheckCommand() {
    return Commands.sequence(
        // Check for hardware errors
        Commands.runOnce(
            () -> {
              REVLibError error = armMotor.getLastError();
              if (error != REVLibError.kOk) {
                addError("Arm motor error: " + error.name());
              } else {
                addInfo("Arm motor contains no errors");
              }
            }),

        // Checks Ground Intake Motor
        moveToPosition(ArmConstants.armL1Position),
        Commands.waitSeconds(MiscellaneousConstants.prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(getPosition()) <= 1e-4) {
                addError("Arm Motor is not moving");
              } else {
                addInfo("Arm Motor is moving");
                if ((Math.abs(ArmConstants.armL1Position) - getPrematchPosition()) > 0.1) {
                  addError("Arm Motor is not at L1 postion");
                  // We just put a fake range for now; we'll update this later on
                } else {
                  addInfo("Arm Motor is at the L1 position");
                }
              }
            }),
        moveToPosition(ArmConstants.armBottomPosition),
        Commands.runOnce(
            () -> {
              if (Math.abs(getPosition()) <= 1e-4) {
                addError("Arm Motor is not moving");
              } else {
                addInfo("Arm Motor is moving");
                if ((Math.abs(ArmConstants.armBottomPosition) - getPrematchPosition()) > 0.1) {
                  addError("Arm Motor is not at bottom postion");
                  // We just put a fake range for now; we'll update this later on
                } else {
                  addInfo("Arm Motor is at the bottom position");
                }
              }
            }),
        moveToPosition(ArmConstants.armTopPosition),
        Commands.runOnce(
            () -> {
              if (Math.abs(getPosition()) <= 1e-4) {
                addError("Arm Motor is not moving");
              } else {
                addInfo("Arm Motor is moving");
                if ((Math.abs(ArmConstants.armL1Position) - getPrematchPosition()) > 0.1) {
                  addError("Arm Motor is not at top (default) postion");
                  // We just put a fake range for now; we'll update this later on
                } else {
                  addInfo("Arm Motor is at the top (default) position");
                }
              }
            }));
  }
}
