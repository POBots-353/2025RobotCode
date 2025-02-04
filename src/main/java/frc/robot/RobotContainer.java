// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LogUtil;
import frc.robot.util.PersistentSendableChooser;

public class RobotContainer {
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final Indexer indexer = new Indexer();
  private final Outtake outtake = new Outtake();
  private final GroundIntake groundIntake = new GroundIntake();
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private PersistentSendableChooser<String> batteryChooser;
  private SendableChooser<Command> autoChooser;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandJoystick operatorStick = new CommandJoystick(1);

  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  private final PowerDistribution powerDistribution = new PowerDistribution();

  private Trigger intakeLaserBroken = new Trigger(groundIntake::intakeLaserBroken);
  private Trigger outtakeLaserBroken = new Trigger(outtake::outtakeLaserBroken);

  private Trigger buttonTrigger = new Trigger(elevator::buttonPressed);
  private Trigger armMode = operatorStick.button(OperatorConstants.armModeButton);

  // Just put a bunch of instantcommands as placeholders for now
  Command outtakePrematch = new InstantCommand();
  Command algaeIntakePrematch = new InstantCommand();
  Command armPrematch = new InstantCommand();
  Command elevatorPrematch = new InstantCommand();
  Command groundIntakePrematch = groundIntake.buildPrematch();
  Command indexerPrematch = indexer.buildPrematch();
  Command swervePrematch = new InstantCommand();

  public RobotContainer() {

    configureDriverBindings();
    configureOperatorBindings();
    configureAutoChooser();
    configureBatteryChooser();

    NamedCommands.registerCommand(
        "ground intake", groundIntake.runIntake().withTimeout(3.0).asProxy());
    NamedCommands.registerCommand("ground intake", groundIntake.stop());
    NamedCommands.registerCommand(
        "outtake", outtake.runOuttake().withTimeout(3.0).asProxy());
    NamedCommands.registerCommand(
        "stop outtake", Commands.run(() -> outtake.stop()));
    NamedCommands.registerCommand(
        "set arm to L4", elevator.moveToPosition(ElevatorConstants.L4Height).asProxy());
    NamedCommands.registerCommand(
        "set arm to humanIntake", elevator.moveToPosition(1.5).withTimeout(3.0).asProxy());
    NamedCommands.registerCommand(
        "set arm home position", elevator.homeElevator().withTimeout(3.0).asProxy());
    NamedCommands.registerCommand(
        "set Indexer", indexer.runIndexer().withTimeout(3.0).asProxy());
    NamedCommands.registerCommand(
        "stop indexer", indexer.stop());

    SmartDashboard.putData("Power Distribution", powerDistribution);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    intakeLaserBroken
        .whileTrue(indexer.runIndexer())
        .onFalse(
            Commands.race(Commands.waitUntil(outtakeLaserBroken), Commands.waitSeconds(4))
                .andThen(indexer::stopIndexer));
  }

  private void configureDriverBindings() {
    Trigger slowMode = driverController.leftBumper();

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

    driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController
        .a()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -driverController.getLeftY(), -driverController.getLeftX()))));

    // driverController.L1().whileTrue(drivetrain.ReefAlign(true));
    // driverController.R1().whileTrue(drivetrain.ReefAlign(false));

    // driverController.R2().whileTrue(new TurnToReef(drivetrain));
    driverController.leftTrigger().whileTrue(drivetrain.humanPlayerAlign());

    // driverController
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.sequence(
    //             drivetrain.pathFindToSetup(),
    //             new TurnToReef(drivetrain),
    //             Commands.waitSeconds(.08),
    //             drivetrain.ReefAlign(true)));
    // driverController
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.sequence(
    //             drivetrain.pathFindToSetup(),
    //             new TurnToReef(drivetrain),
    //             Commands.waitSeconds(.08),
    //             drivetrain.ReefAlign(false)));

    driverController.rightBumper().whileTrue(drivetrain.ReefAlignNoVision(true));

    driverController.rightTrigger().whileTrue(drivetrain.ReefAlignNoVision(false));

    // reset the field-centric heading on left bumper press
    driverController
        .back()
        .and(driverController.start())
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));

    // drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureElevatorBindings() {
    operatorStick
        .button(OperatorConstants.L4HeightButton)
        .and(armMode.negate())
        .onTrue(elevator.moveToPosition(ElevatorConstants.L4Height));
    operatorStick
        .button(OperatorConstants.L3HeightButton)
        .and(armMode.negate())
        .onTrue(elevator.moveToPosition(ElevatorConstants.L3Height));
    operatorStick
        .button(OperatorConstants.L2HeightButton)
        .and(armMode.negate())
        .onTrue(elevator.moveToPosition(ElevatorConstants.L2Height));

    operatorStick
        .button(OperatorConstants.elevatorDownButton)
        .and(armMode.negate())
        .onTrue(elevator.moveToPosition(0));

    operatorStick
        .button(OperatorConstants.homeElevatorButon)
        .and(armMode.negate())
        .whileTrue(elevator.homeElevator())
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

    operatorStick
        .button(OperatorConstants.elevatorManualDown)
        .and(armMode.negate())
        .whileTrue(elevator.downSpeed(.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

    operatorStick
        .button(OperatorConstants.elevatorManualUp)
        .and(armMode.negate())
        .whileTrue(elevator.upSpeed(.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));
  }

  private void configureArmBindings() {
    operatorStick
        .button(OperatorConstants.groundIntakeButton)
        .and(armMode)
        .whileTrue(groundIntake.runIntake())
        .onFalse(groundIntake.stop());

    operatorStick
        .button(OperatorConstants.armManualOuttakeButton)
        .and(armMode)
        .whileTrue(groundIntake.run(groundIntake::manualOuttake))
        .onFalse(groundIntake.stop());

    // Button to raise arm manual up

    // button to raise arm manual down

    // arm to pick up button

    // arm to L1 height button
  }

  private void configureOuttakeBindings() {
    operatorStick
        .button(OperatorConstants.outtakeButton)
        .and(armMode.negate())
        .whileTrue(outtake.runOuttake())
        .onFalse(outtake.stopOuttakeMotor());
  }

  private void configureIndexerBindings() {
    operatorStick
        .button(OperatorConstants.indexerButton)
        .and(armMode.negate())
        .whileTrue(
            indexer
                .runIndexer()
                .alongWith(outtake.runOuttake())
                .unless(outtakeLaserBroken)
                .until(outtakeLaserBroken))
        .onFalse(indexer.stop().alongWith(outtake.stopOuttakeMotor()));

    // button to outtake indexer

  }

  private void configureAlgaeIntakeBindings() {
    // button for algae intake up
    // button for algae intake down
  }

  private void configureOperatorBindings() {
    configureAlgaeIntakeBindings();
    configureArmBindings();
    configureElevatorBindings();
    configureIndexerBindings();
    configureOuttakeBindings();

    // starting config button

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
