// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Indexer;
import frc.robot.util.LogUtil;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.util.PersistentSendableChooser;

public class RobotContainer {
  private final Elevator elevator = new Elevator();
  private final Indexer indexer = new Indexer();
  private final GroundIntake groundIntake = new GroundIntake();
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private PersistentSendableChooser<String> batteryChooser;
  private SendableChooser<Command> autoChooser;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandJoystick operatorStick = new CommandJoystick(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final PowerDistribution powerDistribution = new PowerDistribution();

  private Trigger intakeLaserBroken = new Trigger(groundIntake::intakeLaserBroken());
  private Trigger outakeLaserBroken = new Trigger(indexer::outakeLaserBroken);
  private Trigger buttonTrigger = new Trigger(elevator::buttonPressed);

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
    configureAutoChooser();
    configureBatteryChooser();

    NamedCommands.registerCommand("Stop ground Intake", groundIntake.runIntake().asProxy());
    NamedCommands.registerCommand("Stop ground Intake", groundIntake.intakeLaserBroken().asProxy());
    NamedCommands.registerCommand("Stop ground Intake", groundIntake.stopGroundIntake().asProxy());

    NamedCommands.registerCommand("Start elevator", elevator.().asProxy());
    NamedCommands.registerCommand("Start elevator", elevator.outakeLaserBroken().asProxy());

    NamedCommands.registerCommand("indexer", indexer.buttonTrigger().asProxy());
    NamedCommands.registerCommand("indexer", indexer.stopIndexer().asProxy());


    SmartDashboard.putData("Power Distribution", powerDistribution);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    // intakeLaserBroken
    //     .whileTrue(indexer.run(indexer::index))
    //     .onFalse(indexer.runOnce(indexer::stopIndexer));

    intakeLaserBroken
        .whileTrue(indexer.runIndexer())
        .onFalse(
            Commands.race(Commands.waitUntil(outakeLaserBroken), Commands.waitSeconds(4))
                .andThen(indexer::stopIndexer));
  }
  
  private void configureDriverBindings() {
    Trigger slowMode = driverController.leftTrigger();

    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            () -> {
              if (slowMode.getAsBoolean()) {
                return SwerveConstants.slowModeMaxTranslationalSpeed;
              }
              return SwerveConstants.maxTranslationalSpeed;
            },
            drivetrain));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController
        .start()
        .and(driverController.back())
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureOperatorBindings() {
    elevator.setDefaultCommand(elevator.downPosition());

    operatorStick
        .button(OperatorConstants.indexerButton)
        .and(intakeLaserBroken.negate())
        .whileTrue(indexer.runIndexer())
        .onFalse(indexer.stop());

    operatorStick
        .button(OperatorConstants.groundIntakeButton)
        .whileTrue(groundIntake.runIntake())
        .onFalse(groundIntake.stop());

    operatorStick
        .button(OperatorConstants.L4HeightButton)
        .whileTrue(elevator.moveToPosition(ElevatorConstants.L4Height));

    operatorStick
        .button(OperatorConstants.homeElevatorButon)
        .whileTrue(elevator.homeElevator())
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

    operatorStick
        .button(OperatorConstants.manualOutakeButton)
        .whileTrue(groundIntake.run(groundIntake::manualOutake))
        .onFalse(groundIntake.stop());

    operatorStick
        .button(OperatorConstants.manualFeedButton)
        .and(buttonTrigger)
        .whileTrue(groundIntake.run(groundIntake::feedToIndexer))
        .onFalse(groundIntake.stop());
  }

  private void configureAutoChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption(
        "[SysID] Quasistatic Steer Forward", drivetrain.sysIdQuasistaticSteer(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Quasistatic Steer Reverse", drivetrain.sysIdQuasistaticSteer(Direction.kReverse));
    autoChooser.addOption(
        "[SysID] Dynamic Steer Forward", drivetrain.sysIdDynamicSteer(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Dynamic Steer Reverse", drivetrain.sysIdDynamicSteer(Direction.kReverse));

    autoChooser.addOption(
        "[SysID] Quasistatic Translation Forward",
        drivetrain.sysIdQuasistaticTranslation(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Quasistatic Translation Reverse",
        drivetrain.sysIdQuasistaticTranslation(Direction.kReverse));
    autoChooser.addOption(
        "[SysID] Dynamic Translation Forward",
        drivetrain.sysIdDynamicTranslation(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Dynamic Translation Reverse",
        drivetrain.sysIdDynamicTranslation(Direction.kReverse));

    autoChooser.addOption(
        "[SysID] Quasistatic Rotation Forward",
        drivetrain.sysIdQuasistaticRotation(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Quasistatic Rotation Reverse",
        drivetrain.sysIdQuasistaticRotation(Direction.kReverse));
    autoChooser.addOption(
        "[SysID] Dynamic Rotation Forward", drivetrain.sysIdDynamicRotation(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Dynamic Rotation Reverse", drivetrain.sysIdDynamicRotation(Direction.kReverse));

    autoChooser.addOption(
        "[SysID] Elevator Quasistatic Forward",
        elevator.sysIdQuasistaticElevator(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Elevator Quasistatic Reverse",
        elevator.sysIdQuasistaticElevator(Direction.kReverse));
    autoChooser.addOption(
        "[SysID] Elevator Dynamic Forward", elevator.sysIdDynamicElevator(Direction.kForward));
    autoChooser.addOption(
        "[SysID] Elevator Dynamic Reverse", elevator.sysIdDynamicElevator(Direction.kReverse));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void configureBatteryChooser() {
    batteryChooser = new PersistentSendableChooser<>("Battery Number");

    batteryChooser.addOption("2019 #3", "Daniel");
    batteryChooser.addOption("2020 #2", "Gary");
    batteryChooser.addOption("2022 #1", "Lenny");
    batteryChooser.addOption("2024 #1", "Ian");
    batteryChooser.addOption("2024 #2", "Nancy");
    batteryChooser.addOption("2024 #3", "Perry");
    batteryChooser.addOption("2024 #4", "Quincy");
    batteryChooser.addOption("2024 #5", "Richard");
    batteryChooser.addOption("2025 #1", "Josh");

    if (batteryChooser.getSelectedName() != null && !batteryChooser.getSelectedName().equals("")) {
      LogUtil.recordMetadata("Battery Number", batteryChooser.getSelectedName());
      LogUtil.recordMetadata("Battery Nickname", batteryChooser.getSelected());
    }

    batteryChooser.onChange(
        (nickname) -> {
          LogUtil.recordMetadata("Battery Number", batteryChooser.getSelectedName());
          LogUtil.recordMetadata("Battery Nickname", nickname);
        });

    SmartDashboard.putData("Battery Chooser", batteryChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
