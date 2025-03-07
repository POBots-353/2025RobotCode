// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LogUtil;
import frc.robot.util.PersistentSendableChooser;
import java.util.Set;

@Logged(strategy = Strategy.OPT_IN)
public class RobotContainer {
  @Logged(name = "Swerve")
  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  @Logged(name = "Elevator")
  private final Elevator elevator = new Elevator();

  @Logged(name = "Indexer")
  private final Indexer indexer = new Indexer();

  @Logged(name = "Outtake")
  private final Outtake outtake = new Outtake();

  @Logged(name = "Algae Remover")
  private final AlgaeRemover algaeRemover = new AlgaeRemover();

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private PersistentSendableChooser<String> batteryChooser;
  private SendableChooser<Command> autoChooser;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandJoystick operatorStick = new CommandJoystick(1);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private boolean positionMode = false;
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  private Trigger outtakeLaserBroken = new Trigger(outtake::outtakeLaserBroken);
  private Trigger isPositionMode = new Trigger(() -> positionMode);
  private Trigger buttonTrigger = new Trigger(elevator::buttonPressed);
  private Trigger elevatorIsDown = new Trigger(elevator::elevatorIsDown);
  private Trigger algaeMode = operatorStick.button(OperatorConstants.algaeModeButton);

  public RobotContainer() {
    NamedCommands.registerCommand("Start Indexer", indexer.runIndexer().asProxy());
    NamedCommands.registerCommand("Stop Indexer", indexer.stop().asProxy());
    NamedCommands.registerCommand("PathFindToSetup", drivetrain.pathFindToSetup());
    NamedCommands.registerCommand("Wait for target", drivetrain.waitForTarget());

    NamedCommands.registerCommand("go to HP", drivetrain.humanPlayerAlign());
    NamedCommands.registerCommand(
        "Elevator: L4",
        elevator
            .moveToPosition(ElevatorConstants.L4Height)
            // .onlyIf(outtakeLaserBroken)
            .withTimeout(2.5)
            .asProxy());
    NamedCommands.registerCommand(
        "Elevator: L3",
        elevator
            .moveToPosition(ElevatorConstants.L3Height)
            // .onlyIf(outtakeLaserBroken)
            .withTimeout(4)
            .asProxy());
    NamedCommands.registerCommand("Auto Outtake", outtake.autoOuttake().asProxy());
    NamedCommands.registerCommand("Outtake", outtake.fastOuttake().withTimeout(1.5).asProxy());
    NamedCommands.registerCommand(
        "Elevator: Bottom",
        elevator.downPosition().until(buttonTrigger).withTimeout(3.0).asProxy());
    NamedCommands.registerCommand(
        "OuttakeUntilBeamBreak", outtake.outtakeUntilBeamBreak().withTimeout(2.4).asProxy());
    NamedCommands.registerCommand("AutoAlignLeft", drivetrain.reefAlign(true).withTimeout(3));
    NamedCommands.registerCommand("AutoAlignRight", drivetrain.reefAlign(false).withTimeout(3));

    SmartDashboard.putData("Power Distribution", powerDistribution);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    drivetrain.configureAutoBuilder();

    configureDriverBindings();
    configureOperatorBindings();
    configureAutoChooser();
    configureBatteryChooser();
    configurePrematch();

    // intakeLaserBroken
    //     .whileTrue(indexer.runIndexer())
    //     .onFalse(
    //         Commands.race(Commands.waitUntil(outtakeLaserBroken), Commands.waitSeconds(4))
    //             .andThen(indexer::stopIndexer));
    new Trigger(outtakeLaserBroken)
        .and(() -> !DriverStation.isAutonomous())
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)),
                Commands.waitSeconds(2),
                Commands.runOnce(
                    () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0))));
  }

  private void configurePrematch() {
    Command outtakePrematch = outtake.buildPrematch();
    Command algaeRemoverPrematch = algaeRemover.buildPrematch();
    Command elevatorPrematch = elevator.buildPrematch();
    Command indexerPrematch = indexer.buildPrematch();
    Command swervePrematch = drivetrain.buildPrematch();

    SmartDashboard.putData("Outtake Prematch", outtakePrematch.asProxy());
    SmartDashboard.putData("Algae Remover Prematch", algaeRemoverPrematch.asProxy());
    SmartDashboard.putData("Elevator Prematch", elevatorPrematch.asProxy());
    SmartDashboard.putData("Indexer Prematch", indexerPrematch.asProxy());
    SmartDashboard.putData("Swerve Prematch", swervePrematch.asProxy());
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

    // driverController.square().whileTrue(drivetrain.applyRequest(() -> brake));
    // driverController
    //     .circle()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 point.withModuleDirection(
    //                     new Rotation2d(
    //                         -driverController.getLeftY(), -driverController.getLeftX()))));

    driverController.povUp().onTrue(drivetrain.pathFindToDirection(0));
    driverController.povUpLeft().onTrue(drivetrain.pathFindToDirection(1));
    driverController.povDownLeft().onTrue(drivetrain.pathFindToDirection(2));
    driverController.povDown().onTrue(drivetrain.pathFindToDirection(3));
    driverController.povDownRight().onTrue(drivetrain.pathFindToDirection(4));
    driverController.povUpRight().onTrue(drivetrain.pathFindToDirection(5));

    driverController.rightTrigger().whileTrue(drivetrain.humanPlayerAlign());
    driverController.leftStick().onTrue(Commands.runOnce(() -> positionMode = !positionMode));
    driverController
        .leftBumper()
        .and(isPositionMode.negate())
        .whileTrue(
            Commands.sequence(
                drivetrain.pathFindToSetup(),
                new TurnToReef(drivetrain),
                Commands.waitSeconds(.08),
                drivetrain.reefAlign(true)));
    driverController
        .rightBumper()
        .and(isPositionMode.negate())
        .whileTrue(
            Commands.sequence(
                drivetrain.pathFindToSetup(),
                new TurnToReef(drivetrain),
                Commands.waitSeconds(.08),
                drivetrain.reefAlign(false)));

    driverController.leftBumper().and(isPositionMode).whileTrue(drivetrain.reefAlignNoVision(true));
    driverController
        .rightBumper()
        .and(isPositionMode)
        .whileTrue(drivetrain.reefAlignNoVision(false));

    driverController.x().whileTrue(drivetrain.pathFindForAlgaeRemover());

    // reset the field-centric heading on left bumper press
    driverController
        .back()
        .and(driverController.start())
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);
    // driverController.y().onTrue(elevator.moveToPosition(ElevatorConstants.L4Height));
    // driverController.x().whileTrue(indexer.runIndexer().alongWith(outtake.outtakeUntilBeamBreak()));
    // driverController.b().whileTrue(outtake.autoOuttake());
    // driverController.a().onTrue(elevator.downPosition());
  }

  private void configureElevatorBindings() {
    elevator.setDefaultCommand(elevator.holdPosition());

    // operatorStick
    //     .button(OperatorConstants.L4HeightButton)
    //     .and(algaeMode.negate())
    //     .onTrue(
    //         elevator
    //             .moveToPosition(ElevatorConstants.L4Height)
    //             .andThen(elevator.upSpeed(.1).withTimeout(.25)));

    operatorStick
        .button(OperatorConstants.L4HeightButton)
        .and(algaeMode.negate().and(outtakeLaserBroken))
        .or(
            operatorStick
                .button(OperatorConstants.elevatorOverrideButton)
                .and(operatorStick.button(OperatorConstants.L4HeightButton))
                .and(algaeMode.negate()))
        .onTrue(elevator.moveToPosition(ElevatorConstants.L4Height));
    // .andThen(elevator.upSpeed(.1).withTimeout(.15)));

    operatorStick
        .button(OperatorConstants.L3HeightButton)
        .and(algaeMode.negate().and(outtakeLaserBroken))
        .or(
            operatorStick
                .button(OperatorConstants.elevatorOverrideButton)
                .and(operatorStick.button(OperatorConstants.L3HeightButton))
                .and(algaeMode.negate()))
        .onTrue(elevator.moveToPosition(ElevatorConstants.L3Height));

    // operatorStick
    //     .button(OperatorConstants.L2HeightButton)
    //     .and(algaeMode.negate())
    //     .and(outtakeLaserBroken)
    //     .onTrue(elevator.moveToPosition(ElevatorConstants.L2Height));

    operatorStick
        .button(OperatorConstants.L2HeightButton)
        .and(algaeMode.negate().and(outtakeLaserBroken))
        .or(
            operatorStick
                .button(OperatorConstants.elevatorOverrideButton)
                .and(operatorStick.button(OperatorConstants.L2HeightButton))
                .and(algaeMode.negate()))
        .onTrue(elevator.moveToPosition(ElevatorConstants.L2Height));

    operatorStick
        .button(OperatorConstants.elevatorDownButton)
        .and(algaeMode.negate())
        .onTrue(elevator.downPosition());

    operatorStick
        .button(OperatorConstants.homeElevatorButon)
        .and(algaeMode.negate())
        .onTrue(elevator.homeElevator());

    operatorStick
        .button(OperatorConstants.coralInTheWay)
        .and(algaeMode.negate())
        .onTrue(
            new DeferredCommand(
                () -> {
                  double newTarget =
                      Units.inchesToMeters(
                          elevator.getPositionInches() + ElevatorConstants.coralInTheWayAdd);
                  return elevator.moveToPosition(newTarget);
                },
                Set.of(elevator)));

    operatorStick
        .button(OperatorConstants.elevatorManualDown)
        .and(algaeMode.negate())
        .whileTrue(elevator.downSpeed(0.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

    operatorStick
        .button(OperatorConstants.elevatorManualUp)
        .and(algaeMode.negate().and(outtakeLaserBroken))
        .or(
            operatorStick
                .button(OperatorConstants.elevatorOverrideButton)
                .and(operatorStick.button(OperatorConstants.elevatorManualUp))
                .and(algaeMode.negate()))
        .whileTrue(elevator.upSpeed(0.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

    operatorStick
        .button(OperatorConstants.algaeHighPosition)
        .and(algaeMode)
        .onTrue(
            elevator
                .moveToPosition(ElevatorConstants.AlgaeHighHeight)
                .alongWith(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition))
                .alongWith(outtake.fastOuttake()));

    operatorStick
        .button(OperatorConstants.algaeLowPosition)
        .and(algaeMode)
        .onTrue(
            elevator
                .moveToPosition(ElevatorConstants.AlgaeLowHeight)
                .alongWith(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition))
                .alongWith(outtake.fastOuttake()));

    // operatorStick
    //     .button(OperatorConstants.L4HeightButton)
    //     .and(algaeMode)
    //     .onTrue(
    //         elevator
    //             .moveToPosition(ElevatorConstants.maxHeight)
    //             .alongWith(algaeRemover.run(() -> algaeRemover.algaeRemoverUp())))
    //     .onFalse(algaeRemover.runOnce(() -> algaeRemover.stopAlgaeRemover()));
  }

  // private void configureArmBindings() {
  // arm.setDefaultCommand(arm.moveToPosition(ArmConstants.armL1Position));

  // operatorStick
  //     .button(OperatorConstants.groundIntakeButton)
  //     .and(algaeMode)
  //     .whileTrue(groundIntake.runIntake())
  //     .onFalse(groundIntake.stop());

  // operatorStick
  //     .button(OperatorConstants.armManualOuttakeButton)
  //     .and(algaeMode)
  //     .whileTrue(groundIntake.run(groundIntake::manualOuttake))
  //     .onFalse(groundIntake.stop());

  // // Button to raise arm manual up

  // // button to raise arm manual down
  // operatorStick
  //     .button(OperatorConstants.armPickupHeightButton)
  //     .and(algaeMode)
  //     .onTrue(arm.moveToPosition(ArmConstants.armBottomPosition));

  // operatorStick
  //     .button(OperatorConstants.armL1HeightButton)
  //     .and(algaeMode)
  //     .onTrue(arm.moveToPosition(ArmConstants.armL1Position));
  // }

  private void configureOuttakeBindings() {
    operatorStick
        .button(OperatorConstants.indexerButton)
        .and(algaeMode)
        .onTrue(outtake.reverseOuttake())
        .onFalse(outtake.stopOuttakeMotor());

    operatorStick
        .button(OperatorConstants.outtakeButton)
        .and(algaeMode.negate())
        .onTrue(outtake.fastOuttake())
        .onFalse(outtake.stopOuttakeMotor());
  }

  private void configureIndexerBindings() {
    operatorStick
        .button(OperatorConstants.indexerButton)
        .and(algaeMode.negate())
        .and(elevatorIsDown)
        .whileTrue(indexer.runIndexer())
        .onFalse(indexer.stop());

    operatorStick
        .button(OperatorConstants.indexerButton)
        .and(algaeMode.negate())
        .whileTrue(outtake.outtakeUntilBeamBreak())
        .onFalse(outtake.stopOuttakeMotor());

    operatorStick
        .button(OperatorConstants.reverseIndexerButton)
        .whileTrue(indexer.reverseIndexer())
        .onFalse(indexer.stop());
  }

  private void configureAlgaeRemoverBindings() {
    operatorStick
        .button(OperatorConstants.startingConfigButton)
        // .and(algaeMode)
        .whileTrue(
            elevator
                .moveToPosition(ElevatorConstants.AlgaeHighHeight)
                // .andThen(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition)))
                .andThen(algaeRemover.run(() -> algaeRemover.algaeRemoverDown()).withTimeout(2.1)))
        .onFalse(
            elevator
                .downPosition()
                .withTimeout(3)
                // .alongWith(algaeRemover.run(() -> algaeRemover.algaeRemoverUp()).withTimeout(1))
                .andThen(
                    elevator
                        .runOnce(elevator::stopElevator)
                        .alongWith(
                            algaeRemover
                                .run(() -> algaeRemover.algaeRemoverUp())
                                .withTimeout(.5))));

    //         Commands.parallel(
    //             Commands.runOnce(() -> outtake.fastOuttake()),
    //             elevator.upSpeed(0.075))))
    // .onFalse(Commands.runOnce(() -> outtake.stop()));

    // operatorStick
    //     .button(OperatorConstants.elevatorManualDown)
    //     .and(algaeMode)
    //     .onTrue(
    //         Commands.sequence(
    //             elevator.moveToPosition(ElevatorConstants.AlgaeHighHeight),
    //             Commands.parallel(
    //                 Commands.runOnce(() -> outtake.fastOuttake()),
    //                 algaeRemogit ver.moveToPosition(AlgaeRemoverConstants.intakePosition),
    //                 elevator.upSpeed(0.075))))
    //     .onFalse(Commands.runOnce(() -> outtake.stop()));

    operatorStick
        .button(OperatorConstants.elevatorManualUp)
        .and(algaeMode)
        .whileTrue(algaeRemover.run(() -> algaeRemover.algaeRemoverUp()))
        .onFalse(algaeRemover.runOnce(() -> algaeRemover.stopAlgaeRemover()));
    operatorStick
        .button(OperatorConstants.elevatorManualDown)
        .and(algaeMode)
        .whileTrue(algaeRemover.run(() -> algaeRemover.algaeRemoverDown()))
        .onFalse(algaeRemover.runOnce(() -> algaeRemover.stopAlgaeRemover()));

    // operatorStick
    //     .button(OperatorConstants.algaeGroundIntakeButton)
    //     .and(algaeMode)
    //     .onTrue(
    //         algaeRemover
    //             .moveToPosition(AlgaeRemoverConstants.intakePosition)
    //             .alongWith(outtake.fastOuttake()))
    //     .onFalse(
    //         outtake
    //             .stopOuttakeMotor()
    //             .alongWith(algaeRemover.runOnce(() -> algaeRemover.stopAlgaeRemover())));

    // operatorStick
    //     .button(OperatorConstants.algaeGroundOutButton)
    //     .and(algaeMode)
    //     .onTrue(algaeRemover.moveToPosition(AlgaeRemoverConstants.topPosition));
  }

  private void configureOperatorBindings() {
    configureAlgaeRemoverBindings();
    // configureArmBindings();
    configureElevatorBindings();
    configureIndexerBindings();
    configureOuttakeBindings();

    // operatorStick
    //     .button(OperatorConstants.startingConfigButton)
    //     .whileTrue(
    //         elevator.downPosition().alongWith(arm.moveToPosition(ArmConstants.armTopPosition)))
    //     .onFalse(elevator.runOnce(elevator::stopElevator).alongWith(arm.runOnce(arm::stopArm)));
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

    batteryChooser.initializeFromPreferences();

    Commands.sequence(
            Commands.waitUntil(DriverStation::isDSAttached),
            Commands.waitSeconds(5),
            Commands.runOnce(
                () -> {
                  if (batteryChooser.getSelectedName() != null
                      && !batteryChooser.getSelectedName().equals("")) {
                    LogUtil.recordMetadata("Battery Number", batteryChooser.getSelectedName());
                    LogUtil.recordMetadata("Battery Nickname", batteryChooser.getSelected());
                  }
                }))
        .ignoringDisable(true)
        .withName("Battery Logger")
        .schedule();

    batteryChooser.onChange(
        (nickname) -> {
          LogUtil.recordMetadata("Battery Number", batteryChooser.getSelectedName());
          LogUtil.recordMetadata("Battery Nickname", nickname);
        });

    SmartDashboard.putData("Battery Chooser", batteryChooser);
  }

  public void stopIfBeamBroken() {
    if (outtake.outtakeLaserBroken() && indexer.getSpeed() > 0.0) {
      outtake.stop();
    }
    return;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
