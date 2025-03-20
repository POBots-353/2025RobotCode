// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.ExpandedSubsystem;
import java.util.function.DoubleSupplier;

@Logged(strategy = Strategy.OPT_IN)
public class Elevator extends ExpandedSubsystem {
  /** Creates a new Elevator. */
  private TalonFX elevatorMainMotor;

  private TalonFX elevatorFollowerMotor;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private DigitalInput buttonSwitch = new DigitalInput(ElevatorConstants.buttonSwitchID);
  private boolean isZeroed = false;
  private Alert elevatorAlert;
  private boolean lastButtonState = false;
  private Debouncer buttonDebouncer = new Debouncer(0.5);
  // private Debouncer elevatorDebouncer = new Debouncer(0.353);
  private Debouncer zeroedDebouncer = new Debouncer(2.5);

  private double positionTolerance = Units.inchesToMeters(0.353);

  private StatusSignal<Angle> elevatorMainPosition;
  private StatusSignal<Angle> elevatorFollowerPosition;

  private final SysIdRoutine elevatorSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1.0).per(Second), // Use default ramp rate (1 V/s)
              Volts.of(4.0), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdElevator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                elevatorMainMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
                elevatorFollowerMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
              },
              log -> {
                log.motor("Main")
                    .angularVelocity(
                        Rotations.per(Minute)
                            .of(elevatorMainMotor.getVelocity().getValueAsDouble()))
                    .angularPosition(
                        Rotations.of(elevatorMainMotor.getVelocity().getValueAsDouble()));
                log.motor("Follower")
                    .angularVelocity(
                        Rotations.per(Minute)
                            .of(elevatorFollowerMotor.getVelocity().getValueAsDouble()))
                    .angularPosition(
                        Rotations.of(elevatorFollowerMotor.getVelocity().getValueAsDouble()));
              },
              this));

  private ElevatorSim elevatorSim;

  public Elevator() {
    elevatorMainMotor = new TalonFX(ElevatorConstants.elevatorMainMotorID);
    elevatorFollowerMotor = new TalonFX(ElevatorConstants.elevatorFollowerMotorID);
    // follower = new Follower(ElevatorConstants.elevatorMainMotorID, false);
    elevatorMainMotor.getConfigurator().apply(ElevatorConstants.elevatorConfigs);
    elevatorFollowerMotor.getConfigurator().apply(ElevatorConstants.elevatorConfigs);

    elevatorMainPosition = elevatorMainMotor.getPosition();
    elevatorFollowerPosition = elevatorFollowerMotor.getPosition();

    // elevatorFollowerMotor.setControl(follower);

    elevatorAlert = new Alert("Elevator is not Zeroed!", AlertType.kWarning);
    // elevatorMainMotor.setPosition(0.0);
    // elevatorFollowerMotor.setPosition(0.0);

    if (RobotBase.isSimulation()) {
      elevatorSim =
          new ElevatorSim(
              DCMotor.getKrakenX60(2),
              ElevatorConstants.elevatorGearRatio,
              Units.lbsToKilograms(20),
              ElevatorConstants.sprocketDiameter / 2,
              ElevatorConstants.minHeight,
              ElevatorConstants.maxHeight,
              true,
              0);
    }
  }

  public boolean buttonPressed() {
    return !buttonSwitch.get();
  }

  public Command homeElevator() {
    return downSpeed(0.1)
        .until(() -> buttonDebouncer.calculate(buttonPressed()))
        .unless(() -> buttonDebouncer.calculate(buttonPressed()))
        .finallyDo(
            () ->
                runOnce(
                    () -> {
                      stopElevator();
                      elevatorMainMotor.setPosition(0, 1);
                      elevatorFollowerMotor.setPosition(0, 1);
                    }))
        .withName("Home elevator");
  }

  public void stopElevator() {
    elevatorMainMotor.set(0);
    elevatorFollowerMotor.set(0);
  }

  public boolean elevatorIsDown() {
    return Units.metersToInches(elevatorMainPosition.getValueAsDouble()) < 3.53;
  }

  public Command upSpeed(double speed) {
    return run(() -> {
          elevatorMainMotor.set(speed);
          elevatorFollowerMotor.set(speed);
        })
        .withName("Elevator Manual Up");
  }

  public Command downSpeed(double speed) {
    return run(() -> {
          elevatorMainMotor.set(-speed);
          elevatorFollowerMotor.set(-speed);
        })
        .until(() -> buttonDebouncer.calculate(buttonPressed()))
        .unless(() -> buttonDebouncer.calculate(buttonPressed()))
        .withName("Elevator Manual Down");
  }

  public Command moveToPosition(double height) {
    // double h = height + Units.inchesToMeters(0.2);
    return run(() -> {
          elevatorMainMotor.setControl(motionMagicRequest.withPosition(height));
          elevatorFollowerMotor.setControl(motionMagicRequest.withPosition(height));
        })
        .until(() -> (atSetpoint(height)))
        // .onlyIf(() -> isZeroed)
        .withName("Move to " + height + " meters");
    // .finallyDo(this::holdPosition);
  }

  public Command moveToSuppliedPosition(DoubleSupplier h) {
    return run(() -> {
          double height = h.getAsDouble() + Units.inchesToMeters(1.353);
          elevatorMainMotor.setControl(motionMagicRequest.withPosition(height));
          elevatorFollowerMotor.setControl(motionMagicRequest.withPosition(height));
        })
        .until(() -> (atSetpoint(h.getAsDouble())))
        // .onlyIf(() -> isZeroed)
        .withName("Move to Supplied Height");
    // .finallyDo(this::holdPosition);
  }

  public Command downPosition() {
    return run(() -> {
          elevatorMainMotor.setControl(
              motionMagicRequest.withPosition(ElevatorConstants.downHeight));
          elevatorFollowerMotor.setControl(
              motionMagicRequest.withPosition(ElevatorConstants.downHeight));
        })
        .until(() -> elevatorMainPosition.getValueAsDouble() < Units.inchesToMeters(1))
        .andThen(downSpeed(.02).until(() -> buttonDebouncer.calculate(buttonPressed())))
        .withName("Down Position");
  }

  public boolean atSetpoint(double targetHeight) {
    return Math.abs(targetHeight - elevatorMainPosition.getValueAsDouble()) < positionTolerance;
  }

  public boolean atSetHeight() {
    return Math.abs(elevatorMainPosition.getValueAsDouble() - motionMagicRequest.Position)
        < positionTolerance;
    // && Math.abs(elevatorMainMotor.getVelocity().getValueAsDouble()) < 0.353;
  }

  public Command holdPosition() {
    return startRun(
            () -> {
              motionMagicRequest.Position = elevatorMainPosition.getValueAsDouble();
              SmartDashboard.putNumber(
                  "ELEVAOTR POSTION",
                  Units.metersToInches(elevatorMainPosition.getValueAsDouble()));
              elevatorMainMotor.setControl(motionMagicRequest);
              elevatorFollowerMotor.setControl(motionMagicRequest);
            },
            () -> {
              elevatorMainMotor.setControl(motionMagicRequest);
              elevatorFollowerMotor.setControl(motionMagicRequest);
            })
        .withName("Elevator Hold");
  }

  public Command sysIdQuasistaticElevator(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamicElevator(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.dynamic(direction);
  }

  public double getPositionMeters() {
    return elevatorMainPosition.getValueAsDouble();
  }

  public double getPositionInches() {
    return Units.metersToInches(elevatorMainPosition.getValueAsDouble());
  }

  @Logged(name = "3D Pose")
  public Pose3d[] getPose3d() {
    double height = elevatorMainPosition.getValueAsDouble();

    return new Pose3d[] {
      new Pose3d(0, 0, height, Rotation3d.kZero), new Pose3d(0, 0, height * 2, Rotation3d.kZero),
    };
  }

  @Override
  public void periodic() {
    elevatorMainPosition.refresh();
    elevatorFollowerPosition.refresh();

    SmartDashboard.putBoolean("Elevator/AtSetHeight", atSetHeight());
    SmartDashboard.putNumber(
        "Elevator/MotorVelocity", elevatorMainMotor.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber(
        "Elevator/Main Stage 1 Position",
        Units.metersToInches(elevatorMainPosition.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Elevator/Main Carriage Position",
        Units.metersToInches(elevatorMainPosition.getValueAsDouble()) * 2);
    SmartDashboard.putNumber(
        "Elevator/Follower Stage 1 Position",
        Units.metersToInches(elevatorFollowerPosition.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Elevator/Follower Carriage Position",
        Units.metersToInches(elevatorFollowerPosition.getValueAsDouble()) * 2);
    SmartDashboard.putBoolean("Elevator/Button Pressed", buttonPressed());

    boolean currentButtonState = buttonPressed();

    if (currentButtonState && !lastButtonState) {
      elevatorMainMotor.setPosition(0, 0);
      elevatorFollowerMotor.setPosition(0, 0);

      isZeroed = true;
    }

    lastButtonState = currentButtonState;

    if (isZeroed
        && zeroedDebouncer.calculate(
            Units.metersToInches(elevatorMainPosition.getValueAsDouble()) < 1)
        && !buttonPressed()) {
      isZeroed = false;
    }

    elevatorAlert.set(!isZeroed);
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimState mainSimState = elevatorMainMotor.getSimState();
    TalonFXSimState followerSimState = elevatorFollowerMotor.getSimState();

    mainSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    elevatorSim.setInputVoltage(mainSimState.getMotorVoltage());

    elevatorSim.update(0.020);

    mainSimState.setRawRotorPosition(
        elevatorSim.getPositionMeters() * ElevatorConstants.sensorToMechanismRatio);
    mainSimState.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.sensorToMechanismRatio);

    followerSimState.setRawRotorPosition(
        elevatorSim.getPositionMeters() * ElevatorConstants.sensorToMechanismRatio);
    followerSimState.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.sensorToMechanismRatio);
  }

  @Override
  public Command getPrematchCheckCommand() {
    Command preMatchL4 =
        Commands.parallel(
            moveToPosition(ElevatorConstants.L4Height),
            Commands.sequence(
                Commands.waitTime(Seconds.of(2)),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(getPositionInches()) <= 1e-4) {
                        addError("Elevator is not moving");
                      } else {
                        if (!atSetpoint(ElevatorConstants.L4Height)) {
                          addError("Elevator is not near L4 height");
                          // We just put a fake range for now; we'll update this later on
                        } else {
                          addInfo("Elevator successfully reached L4 height");
                        }
                      }
                    })));

    Command preMatchL3 =
        Commands.parallel(
            moveToPosition(ElevatorConstants.L3Height),
            Commands.sequence(
                Commands.waitTime(Seconds.of(1.5)),
                Commands.runOnce(
                    () -> {
                      if (!atSetpoint(ElevatorConstants.L3Height)) {
                        addError("Elevator is not near L3 height");
                      } else {
                        addInfo("Elevator successfully reached L3 height");
                      }
                    })));

    Command preMatchL2 =
        Commands.parallel(
            moveToPosition(ElevatorConstants.L2Height),
            Commands.sequence(
                Commands.waitTime(Seconds.of(1.5)),
                Commands.runOnce(
                    () -> {
                      if (Math.abs(getPositionInches()) <= 1e-4) {
                        addError("Elevator is not moving");
                      } else {
                        addInfo("Elevator is moving");
                        if (!atSetpoint(ElevatorConstants.L2Height)) {
                          addError("Elevator is not near L2 height");
                        } else {
                          addInfo("Elevator successfully reached L2 height");
                        }
                      }
                    })));

    Command preMatchDown =
        Commands.parallel(
            downPosition(),
            Commands.sequence(
                Commands.waitSeconds(2),
                Commands.runOnce(
                    () -> {
                      if (getPositionMeters() > ElevatorConstants.downHeight) {
                        addError("Elevator did not lower");
                      } else if (!buttonPressed()) {
                        addError("Button is not pressed");
                      } else {
                        addInfo("Elevator has lowered");
                      }
                    })));
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              addInfo("CHECK BOTH MOTORS FOR HAZARD LIGHTS");
              Elastic.sendNotification(
                  new Elastic.Notification()
                      .withLevel(NotificationLevel.WARNING)
                      .withTitle("CHECK MOTORS")
                      .withDescription("CHECK BOTH MOTORS FOR HAZARD LIGHTS")
                      .withDisplaySeconds(5));
            }),
        Commands.waitSeconds(5),
        preMatchL4.asProxy().withName("Pre-Match Move to L4"),
        preMatchDown.asProxy().withName("Pre-Match Lower"),
        preMatchL3.asProxy().withName("Pre-Match Move to L3"),
        preMatchDown.asProxy().withName("Pre-Match Lower"),
        preMatchL2.asProxy().withName("Pre-Match Move to L2"),
        preMatchDown.asProxy().withName("Pre-Match Lower"),
        preMatchL2.asProxy().withName("Pre-Match Move to L2"),
        preMatchL3.asProxy().withName("Pre-Match Move to L3"),
        preMatchL4.asProxy().withName("Pre-Match Move to L4"),
        preMatchDown.asProxy().withName("Pre-Match Lower"));
  }
}
