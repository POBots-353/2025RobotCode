// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToAlgae;
import frc.robot.commands.TurnToReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeRemover;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LogUtil;
import frc.robot.util.PersistentSendableChooser;
import java.util.function.DoubleSupplier;

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

  @Logged(name = "Algae Intake")
  private final AlgaeIntake algaeIntake = new AlgaeIntake();

  //   private final Climber climber = new Climber();

  private final LEDs led = new LEDs();

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private PersistentSendableChooser<String> batteryChooser;
  private SendableChooser<Command> autoChooser;

  private final CommandXboxController driverController = new CommandXboxController(1);
  //   private final CommandJoystick operatorStick = new CommandJoystick(1);
  private final CommandXboxController operatorController = new CommandXboxController(0);

  private boolean isAlgaeAuto = true;

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  //   private boolean positionMode = false;
  private boolean prepareElevator = false;

  private boolean removerLocked = false;

  private boolean doubleScoreActive = false;
  private DoubleSupplier selectedHeight = () -> 0.0;
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  private Trigger outtakeLaserBroken = new Trigger(outtake::outtakeLaserBroken);
  private Trigger algaeLaserBroken = new Trigger(algaeRemover::algaeLaserBroken);

  private Trigger removerIsLocked = new Trigger(() -> removerLocked);

  private Trigger prepareElevatorUp = new Trigger(() -> prepareElevator);
  //   private Trigger isPositionMode = new Trigger(() -> positionMode);
  private Trigger buttonTrigger = new Trigger(elevator::buttonPressed);
  private Trigger elevatorIsDown = new Trigger(elevator::elevatorIsDown);
  private Trigger atValidReefPose = new Trigger(drivetrain::atValidReefPose);
  private Trigger atElevatorHeight = new Trigger(elevator::atSetHeight);

  private Trigger algaeModeButton = operatorController.leftBumper();
  private Trigger elevatorL4Button = operatorController.b();
  private Trigger elevatorL3Button = operatorController.y();
  private Trigger elevatorL2Button = operatorController.x();
  private Trigger elevatorDownButton = operatorController.a();

  private Trigger doubleScoreMode = algaeModeButton.and(operatorController.start().negate());
  private Trigger inBargeRange = new Trigger(drivetrain::inAlgaeRange);
  private Trigger timeToPark = new Trigger(() -> DriverStation.getMatchTime() <= 5);

  public RobotContainer() {
    NamedCommands.registerCommand("Start Indexer", indexer.runIndexer().asProxy());
    NamedCommands.registerCommand("Stop Indexer", indexer.stop().asProxy());
    NamedCommands.registerCommand("PathFindToSetup", drivetrain.pathFindToSetup());
    NamedCommands.registerCommand("Wait for target", drivetrain.waitForTarget());

    NamedCommands.registerCommand("go to Barge", drivetrain.pathFindToBarge());
    NamedCommands.registerCommand("go to HP", drivetrain.humanPlayerAlign());
    NamedCommands.registerCommand(
        "Elevator: L4",
        elevator
            .moveToPosition(ElevatorConstants.L4Height)
            // .onlyIf(outtakeLaserBroken)
            .withTimeout(1.80)
            .asProxy());
    NamedCommands.registerCommand(
        "Elevator: L3",
        elevator
            .moveToPosition(ElevatorConstants.L3Height)
            // .onlyIf(outtakeLaserBroken)
            .withTimeout(4)
            .asProxy());
    NamedCommands.registerCommand(
        "Elevator: L2",
        elevator
            .moveToPosition(ElevatorConstants.L2Height) // .onlyIf(outtakeLaserBroken)
            .withTimeout(4)
            .asProxy());
    NamedCommands.registerCommand("Auto Outtake", outtake.autoOuttake().asProxy());
    NamedCommands.registerCommand("Outtake", outtake.fastOuttake().withTimeout(1.5).asProxy());
    NamedCommands.registerCommand(
        "Elevator: Bottom",
        elevator.downPosition().until(buttonTrigger).withTimeout(3.0).asProxy());
    NamedCommands.registerCommand(
        "OuttakeUntilBeamBreak", outtake.outtakeUntilBeamBreak().withTimeout(0.9).asProxy());

    NamedCommands.registerCommand(
        "IntakeUntilBeamBreak",
        outtake.outtakeUntilBeamBreak().alongWith(indexer.runIndexer()).asProxy());
    NamedCommands.registerCommand("Stop Outtake", outtake.stopOuttakeMotor().asProxy());
    NamedCommands.registerCommand(
        "ZeroAlgaeRemover",
        Commands.runOnce(() -> algaeRemover.resetPosition()).withTimeout(.15).asProxy());

    // NamedCommands.registerCommand(
    //     "ZeroClimber",
    //     Commands.runOnce(() -> climber.zeroPosition()).withTimeout(.15).asProxy());

    NamedCommands.registerCommand("ZeroClimber", Commands.none().withTimeout(.15).asProxy());

    NamedCommands.registerCommand("Turn to reef", new TurnToReef(drivetrain).withTimeout(2));
    NamedCommands.registerCommand("AutoAlignLeft", drivetrain.reefAlign(true).withTimeout(2.8));
    NamedCommands.registerCommand("AutoAlignRight", drivetrain.reefAlign(false).withTimeout(2.8));
    NamedCommands.registerCommand(
        "AutoAutoAlignRight", drivetrain.autoReefAlign(false).withTimeout(2.8));

    NamedCommands.registerCommand(
        "Double Score: 1",
        elevator
            .moveToSuppliedPosition(() -> drivetrain.getAlgaeHeight())
            .withTimeout(.7)
            .andThen(
                algaeRemover
                    .moveToPosition(AlgaeRemoverConstants.autoIntakePosition)
                    .alongWith(algaeIntake.intake()))
            .withTimeout(2)
            .asProxy());
    NamedCommands.registerCommand(
        "Double Score: 2",
        Commands.sequence(
                algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition),
                elevator.moveToSuppliedPosition(() -> ElevatorConstants.L4Height).withTimeout(2),
                outtake.autoOuttake().withTimeout(1))
            .asProxy());
    // private Command startDoubleScoreCommand() {
    //     return algaeRemover
    //         .moveToPosition(AlgaeRemoverConstants.holdPosition)
    //         .alongWith(elevator.moveToSuppliedPosition(() -> drivetrain.getAlgaeHeight()))
    //         .withTimeout(1.5)
    //         .andThen(
    //             algaeRemover
    //                 .moveToPosition(AlgaeRemoverConstants.intakePosition)
    //                 .alongWith(algaeIntake.intake()))
    //         .withName("Startdoublescorecommand");
    //   }

    //   private Command finishDoubleScoreCommand(DoubleSupplier targetHeight) {
    //     return Commands.sequence(
    //             algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition),
    //             elevator.moveToSuppliedPosition(targetHeight).withTimeout(2),
    //             outtake.autoOuttake())
    //         .withName("Finishdoublescorecommand");
    //   }

    NamedCommands.registerCommand(
        "Grab Algae",
        Commands.sequence(
                elevator
                    .moveToSuppliedPosition(() -> drivetrain.getAlgaeHeight())
                    .alongWith(
                        Commands.waitTime(AlgaeRemoverConstants.reefIntakeTimingOffset)
                            .andThen(
                                algaeRemover
                                    .moveToPosition(AlgaeRemoverConstants.autoIntakePosition)
                                    .alongWith(algaeIntake.intake())))
                    .withTimeout(1.3),
                algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition))
            .asProxy());

    NamedCommands.registerCommand("Release Algae", algaeIntake.outtake().withTimeout(2).asProxy());

    NamedCommands.registerCommand(
        "Barge Score: 1",
        elevator.moveToPosition(ElevatorConstants.bargeHeight).withTimeout(1.2).asProxy());

    NamedCommands.registerCommand(
        "Barge Score: 2",
        algaeRemover
            .moveToPosition(AlgaeRemoverConstants.bargeScorePosition)
            .alongWith(Commands.waitSeconds(.1).andThen(algaeIntake.outtake()))
            .withTimeout(.75)
            .asProxy());

    NamedCommands.registerCommand(
        "Barge Score: 3",
        algaeRemover
            .moveToPosition(AlgaeRemoverConstants.holdPosition)
            .alongWith(elevator.downPosition())
            .withTimeout(1.1)
            .asProxy());
    NamedCommands.registerCommand(
        "Algae Remover: HOLD",
        algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition).withTimeout(1.5).asProxy());

    NamedCommands.registerCommand(
        "EnableAlgaeIntakeDefaultCommand", Commands.runOnce(() -> isAlgaeAuto = true).asProxy());

    NamedCommands.registerCommand(
        "DisableAlgaeIntakeDefaultCommand", Commands.runOnce(() -> isAlgaeAuto = false).asProxy());

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

    timeToPark
        .and(() -> !DriverStation.isAutonomous())
        .onTrue(
            Commands.runOnce(
                () -> {
                  driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
                }));

    inBargeRange
        .and(() -> !DriverStation.isAutonomous())
        .onTrue(
            Commands.runOnce(
                () -> {
                  operatorController.getHID().setRumble(RumbleType.kBothRumble, 1);
                  driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                  operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                }));

    outtakeLaserBroken
        .and(() -> !DriverStation.isAutonomous())
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          operatorController.getHID().setRumble(RumbleType.kBothRumble, 1);
                          //   driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
                        }),
                    Commands.waitSeconds(2),
                    Commands.runOnce(
                        () -> {
                          //   driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        }))
                .withName("Outtake Laser Vibration"));

    algaeLaserBroken.onTrue(Commands.runOnce(() -> removerLocked = true));

    prepareElevatorUp
        .onTrue(elevator.moveToPosition(ElevatorConstants.L2Height))
        .onFalse(elevator.downPosition());
    // new Trigger(
    //         () ->
    //             DriverStation.isAutonomous()
    //                 && DriverStation.isEnabled()
    //                 && (DriverStation.getMatchTime() <= 1)
    //                 && DriverStation.getMatchTime() > 0)
    //     .onTrue(Commands.sequence(Commands.waitSeconds(0.50), outtake.fastOuttake()));

    // outtakeLaserBroken.whileTrue(led.run(() ->
    // led.setColor(Color.kGreen)).ignoringDisable(true));

    outtakeLaserBroken.whileTrue(
        led.blink(Color.kGreen)
            .withTimeout(Seconds.of(2))
            .andThen(led.solidColor(Color.kGreen))
            .ignoringDisable(true)
            .withName("LED Laser CAN Blink"));

    doubleScoreMode.whileTrue(led.blink(Color.kPurple).withName("DoubleScore Mode LEDS"));
    doubleScoreMode.onTrue(Commands.runOnce(() -> doubleScoreActive = true));
    // led.setDefaultCommand(led.rainbowScroll().ignoringDisable(true).withName("LED Rainbow
    // Scroll"));
    // led.setDefaultCommand(led.solidColor(Color.kBlack).ignoringDisable(true));

    // Hacky way to get the default LED command to switch
    // new Trigger(DriverStation::isDSAttached).onChange(led.runOnce(() ->
    // {}).ignoringDisable(true));
    new Trigger(DriverStation::isEnabled)
        .onChange(
            Commands.runOnce(
                    () -> {
                      driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
                      operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
                    })
                .ignoringDisable(true));

    led.setDefaultCommand(
        Commands.either(
            // If connected
            led.loadingAnimation(Color.kWhite, 6, Seconds.of(2))
                .ignoringDisable(true)
                .withName("Disconnected Loading"),
            led.scrolling(Color.kGreen, Color.kBlue).ignoringDisable(true),

            // If disconnected,
            DriverStation::isDSAttached));

    // led.setDefaultCommand(
    //     led.elevatorProgress(elevator::getPositionMeters)
    //         .ignoringDisable(true)
    //         .withName("Elevator Progress LED"));
  }

  //   operatorController
  //   .leftTrigger()
  //   .whileTrue(
  //       elevator.moveToSuppliedPosition(() -> drivetrain.getAlgaeHeight())
  //
  // .alongWith(Commands.waitTime(AlgaeRemoverConstants.reefIntakeTimingOffset).andThen(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition)))
  //       .alongWith(algaeRemover.intake()))
  //   .onFalse(
  //           algaeRemover.stop()
  //
  // .alongWith(algaeRemover.moveToPosition(AlgaeRemoverConstants.stowPosition).andThen(elevator.downPosition())));
  private Command startDoubleScoreCommand() {
    return algaeRemover
        .moveToPosition(AlgaeRemoverConstants.holdPosition)
        .alongWith(elevator.moveToSuppliedPosition(() -> drivetrain.getAlgaeHeight()))
        .withTimeout(1.5)
        .andThen(
            algaeRemover
                .moveToPosition(AlgaeRemoverConstants.intakePosition)
                .alongWith(algaeIntake.intake()))
        .withName("Startdoublescorecommand");
  }

  private Command finishDoubleScoreCommand(DoubleSupplier targetHeight) {
    return Commands.sequence(
            algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition),
            elevator.moveToSuppliedPosition(targetHeight).withTimeout(2),
            outtake.autoOuttake())
        .withName("Finishdoublescorecommand");
  }

  private void configurePrematch() {
    Command elevatorPrematch = elevator.buildPrematch();
    Command indexerPrematch = indexer.buildPrematch();
    Command swervePrematch = drivetrain.buildPrematch();

    Command coralIntakePrematch =
        Commands.sequence(
            indexer
                .runIndexer()
                .alongWith(outtake.outtakeUntilBeamBreak())
                .until(outtakeLaserBroken),
            indexer.stop().alongWith(outtake.stopOuttakeMotor()),
            elevator
                .moveToPosition(ElevatorConstants.L2Height)
                .andThen(outtake.autoOuttake().andThen(elevator.downPosition())),
            indexer
                .runIndexer()
                .alongWith(outtake.outtakeUntilBeamBreak())
                .until(outtakeLaserBroken),
            indexer.stop().alongWith(outtake.stopOuttakeMotor()),
            elevator
                .moveToPosition(ElevatorConstants.L3Height)
                .andThen(outtake.autoOuttake().andThen(elevator.downPosition())));

    SmartDashboard.putData(
        "Elevator Prematch", elevatorPrematch.asProxy().withName("Elevator Prematch"));
    SmartDashboard.putData(
        "Indexer Prematch", indexerPrematch.asProxy().withName("Indexer Prematch"));
    SmartDashboard.putData("Swerve Prematch", swervePrematch.asProxy().withName("Swerve Prematch"));
    SmartDashboard.putData(
        "Coral Intake Prematch", coralIntakePrematch.asProxy().withName("Coral Intake Prematch"));
  }

  private void configureDriverBindings() {
    Trigger slowModeButton = driverController.leftTrigger();
    Trigger pathFindToBargeButton = driverController.leftStick();
    // Trigger autoButton = driverController.y();
    Trigger turnToAlgaeButton = driverController.rightStick();
    Trigger pathFindToStationButton = driverController.rightTrigger();
    Trigger leftAlignButton = driverController.leftBumper();
    Trigger rightAlignButton = driverController.rightBumper();
    Trigger zeroOnReefButton = driverController.b();
    Trigger pathFindToL1Button = driverController.x();
    Trigger resetHeadingButton = driverController.start().and(driverController.back());
    // Trigger startClimbButton = driverController.y();
    // Trigger finishClimbButton = driverController.a();

    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            () -> {
              if (slowModeButton.getAsBoolean()) {
                return SwerveConstants.slowModeMaxTranslationalSpeed;
              }
              return SwerveConstants.maxTranslationalSpeed;
            },
            drivetrain));

    pathFindToBargeButton.whileTrue(
        drivetrain
            .pathFindToBarge()
            .andThen(
                new TeleopSwerve(
                    () -> 0.0,
                    driverController::getRightX,
                    () -> 0.0,
                    () -> SwerveConstants.slowModeMaxTranslationalSpeed,
                    drivetrain)));
    // driverController
    //     .rightStick()
    //     .whileTrue(
    //         drivetrain
    //             .pathFindToProcessor()
    //             .andThen(
    //                 new TeleopSwerve(
    //                     () -> 0.0,
    //                     driverController::getLeftX,
    //                     () -> 0.0,
    //                     () -> SwerveConstants.slowModeMaxTranslationalSpeed,
    //                     drivetrain)));

    // autoButton.onTrue(
    //     new InstantCommand(
    //         () -> {
    //           Command autoCommand = autoChooser.getSelected();
    //           autoCommand.schedule();
    //         },
    //         drivetrain,
    //         elevator,
    //         indexer,
    //         algaeIntake,
    //         algaeRemover,
    //         outtake));

    // startClimbButton.whileTrue(climber.moveToSetup()).onFalse(climber.stopWinch());
    // finishClimbButton
    //     .whileTrue(climber.startWinch().onlyIf(() -> climber.atSetpoint()))
    //     .onFalse(climber.stopWinch());

    turnToAlgaeButton.whileTrue(
        new TurnToAlgae(
            driverController::getLeftY,
            driverController::getLeftX,
            () -> {
              if (slowModeButton.getAsBoolean()) {
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

    pathFindToStationButton.whileTrue(drivetrain.humanPlayerAlign());

    // Command outtakeAfterAlign =
    //     Commands.run(
    //             () -> {
    //               if (atValidReefPose.getAsBoolean()
    //                   && atElevatorHeight.getAsBoolean()
    //                   && !elevatorIsDown.getAsBoolean()
    //                   && outtake.outtakeLaserBroken()) {
    //                 Commands.sequence(Commands.waitTime(Seconds.of(0.1)), outtake.autoOuttake())
    //                     .schedule();
    //               }
    //             })
    //         .until(atValidReefPose.and(atElevatorHeight).and(elevatorIsDown.negate()));
    // outtakeAfterAlign.asProxy

    // driverController.R3().onTrue(Commands.runOnce(() -> positionMode = !positionMode));

    // driverController
    //     .b()
    //     .whileTrue(
    //         Commands.repeatingSequence(
    //             indexer
    //                 .runIndexer()
    //                 .until(outtakeLaserBroken)
    //                 .alongWith(outtake.outtakeUntilBeamBreak()),
    //             indexer.stop(),
    //             elevator.moveToPosition(Units.inchesToMeters(12)),
    //             outtake
    //                 .reverseOuttake()
    //                 .until(() -> loopDebouncer.calculate(!outtakeLaserBroken.getAsBoolean())),
    //             elevator.downPosition()))
    //     .onFalse(
    //         indexer
    //             .stop()
    //             .alongWith(elevator.downPosition())
    //             .alongWith(outtake.stopOuttakeMotor()));

    leftAlignButton
        .whileTrue(
            Commands.sequence(
                drivetrain.pathFindToSetup(),
                Commands.parallel(
                    new TurnToReef(drivetrain),
                    Commands.runOnce(() -> prepareElevator = true).onlyIf(outtakeLaserBroken)),
                drivetrain.reefAlign(true)))
        .onFalse(Commands.runOnce(() -> prepareElevator = false));
    // drivetrain.reefAlign(true),
    // Commands.waitUntil(
    //     outtakeLaserBroken
    //         .and(doubleScoreMode.negate())
    //         // .and(atValidReefPose)
    //         .and(elevatorIsDown.negate())
    //         .and(atElevatorHeight)),
    // outtake.autoOuttake()));

    rightAlignButton
        .whileTrue(
            Commands.sequence(
                drivetrain.pathFindToSetup(),
                Commands.parallel(
                    new TurnToReef(drivetrain),
                    Commands.runOnce(() -> prepareElevator = true).onlyIf(outtakeLaserBroken)),
                drivetrain.reefAlign(false)))
        .onFalse(Commands.runOnce(() -> prepareElevator = false));
    // drivetrain.reefAlign(true),
    // Commands.waitUntil(
    //     outtakeLaserBroken
    //         .and(doubleScoreMode.negate())
    //         // .and(atValidReefPose)
    //         .and(elevatorIsDown.negate())
    //         .and(atElevatorHeight)),
    // outtake.autoOuttake()));

    // driverController.leftBumper().and(isPositionMode).whileTrue(drivetrain.reefAlignNoVision(true));
    // driverController.rightBumper().and(isPositionMode).whileTrue(drivetrain.reefAlignNoVision(false));

    pathFindToL1Button.whileTrue(drivetrain.pathFindForL1Score());

    // reset the field-centric heading
    resetHeadingButton.onTrue(
        drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));

    zeroOnReefButton.onTrue(drivetrain.runOnce(drivetrain::zeroYawOnReef));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureElevatorBindings() {
    elevator.setDefaultCommand(
        Commands.either(Commands.none(), elevator.holdPosition(), buttonTrigger));

    Trigger elevatorOverrideButton = operatorController.povLeft();

    Trigger L1Button = operatorController.povRight();
    Trigger manualUpButton = operatorController.povUp();
    Trigger manualDownButton = operatorController.povDown();
    Trigger resetElevatorButton = operatorController.start().and(operatorController.back());

    // Elevator L4
    elevatorL4Button
        .and(outtakeLaserBroken)
        .and(algaeModeButton.negate())
        .or(elevatorOverrideButton.and(elevatorL4Button))
        .onTrue(
            elevator.moveToPosition(ElevatorConstants.L4Height)
            // .alongWith(
            //     Commands.waitSeconds(1.1)
            //         .andThen(
            //             new ConditionalCommand(
            //                 outtake.autoOuttake(),
            //                 Commands.none(),
            //                 () ->
            //                     (driverController.leftBumper().getAsBoolean()
            //                             || driverController.rightBumper().getAsBoolean())
            //                         && outtakeLaserBroken.getAsBoolean())))
            );

    // elevator L3
    elevatorL3Button
        .and(outtakeLaserBroken)
        .and(algaeModeButton.negate())
        .or(elevatorOverrideButton.and(elevatorL3Button))
        .onTrue(
            elevator.moveToPosition(ElevatorConstants.L3Height)
            // .alongWith(
            //     Commands.waitSeconds(.8)
            //         .andThen(
            //             new ConditionalCommand(
            //                 outtake.autoOuttake(),
            //                 Commands.none(),
            //                 () ->
            //                     (driverController.leftBumper().getAsBoolean()
            //                             || driverController.rightBumper().getAsBoolean())
            //                         && outtakeLaserBroken.getAsBoolean())))
            );

    // elevator L2
    elevatorL2Button
        .and(outtakeLaserBroken)
        .and(algaeModeButton.negate())
        .or(elevatorOverrideButton.and(elevatorL2Button))
        .onTrue(
            elevator.moveToPosition(ElevatorConstants.L2Height)
            // .alongWith(
            //     Commands.waitSeconds(.5)
            //         .andThen(
            //             new ConditionalCommand(
            //                 outtake.autoOuttake(),
            //                 Commands.none(),
            //                 () ->
            //                     (driverController.leftBumper().getAsBoolean()
            //                             || driverController.rightBumper().getAsBoolean())
            //                         && outtakeLaserBroken.getAsBoolean())))
            );

    // elevator down height
    elevatorDownButton.onTrue(elevator.downPosition());

    L1Button.onTrue(
        Commands.sequence(
            elevator.moveToPosition(ElevatorConstants.L1Height).withTimeout(1),
            outtake.fastOuttake().alongWith(elevator.upSpeed(1)).withTimeout(.353),
            outtake.stopOuttakeMotor()));

    // home elevator
    resetElevatorButton.and(algaeModeButton.negate()).onTrue(elevator.homeElevator());

    // // coral in the way add
    // operatorController
    //     .povRight()
    //     .onTrue(
    //         new DeferredCommand(
    //             () -> {
    //               double newTarget =
    //                   Units.inchesToMeters(
    //                       elevator.getPositionInches() + ElevatorConstants.coralInTheWayAdd);
    //               return elevator.moveToPosition(newTarget);
    //             },
    //             Set.of(elevator)));
    // elevator manual down
    manualDownButton
        .and(algaeModeButton.negate())
        .whileTrue(elevator.downSpeed(0.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));
    // elevator manual up
    manualUpButton
        .and(algaeModeButton.negate())
        .whileTrue(elevator.upSpeed(0.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

    // operatorController
    //     .button(OperatorConstants.algaeHighPosition)
    //     .onTrue(
    //         elevator
    //             .moveToPosition(ElevatorConstants.AlgaeHighHeight)
    //             .alongWith(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition))
    //             .alongWith(outtake.fastOuttake()));

    // operatorController
    //     .button(OperatorConstants.algaeLowPosition)
    //     .onTrue(
    //         elevator
    //             .moveToPosition(ElevatorConstants.AlgaeLowHeight)
    //             .alongWith(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition))
    //             .alongWith(outtake.fastOuttake()));
  }

  private void configureOuttakeBindings() {
    Trigger outtakeButton = operatorController.leftTrigger();

    outtakeButton.whileTrue(outtake.fastOuttake()).onFalse(outtake.stopOuttakeMotor());
  }

  private void configureIndexerBindings() {
    Trigger indexButton = operatorController.rightTrigger();
    Trigger reverseIndexerButton =
        operatorController.back().and(operatorController.start().negate());

    indexButton
        .and(elevatorIsDown)
        .and(algaeModeButton.negate())
        .whileTrue(indexer.runIndexer().alongWith(outtake.outtakeUntilBeamBreak()))
        .onFalse(indexer.stop().alongWith(outtake.stopOuttakeMotor()));

    reverseIndexerButton.whileTrue(indexer.reverseIndexer()).onFalse(indexer.stop());
  }

  private void configureAlgaeRemoverBindings() {
    algaeIntake.setDefaultCommand(
        Commands.either(
            Commands.none(),
            algaeIntake.slowIntake(),
            () -> DriverStation.isAutonomous() && !isAlgaeAuto));

    algaeRemover.setDefaultCommand(
        Commands.either(
            algaeRemover.bringBackAlgaeRemover(),
            Commands.none(),
            () ->
                !DriverStation.isAutonomous()
                    && buttonTrigger.getAsBoolean()
                    && !removerIsLocked.getAsBoolean()));

    Trigger rezeroAlgaeButton = operatorController.back().and(operatorController.start());
    Trigger algaeUpButton = operatorController.povDown();
    Trigger algaeDownButton = operatorController.povUp();
    Trigger algaeIntakeSequenceButton =
        operatorController
            .start()
            .and(operatorController.back().negate())
            .and(algaeModeButton.negate());
    Trigger bargeScoreSequenceButton = operatorController.leftStick();
    Trigger processorScoreSequenceButton = operatorController.rightStick();
    Trigger algaeFloorButton = operatorController.rightBumper();
    Trigger algaeManualOuttakeButton =
        operatorController.start().and(operatorController.back().negate()).and(algaeModeButton);
    Trigger algaeManualIntakeButton = operatorController.rightTrigger().and(algaeModeButton);

    rezeroAlgaeButton
        .or(algaeUpButton)
        .or(algaeDownButton)
        .or(algaeIntakeSequenceButton)
        .or(bargeScoreSequenceButton)
        .or(processorScoreSequenceButton)
        .or(algaeFloorButton)
        .onTrue(Commands.runOnce(() -> removerLocked = false));

    // zero algaeintake
    rezeroAlgaeButton
        .and(algaeModeButton)
        .onTrue(Commands.runOnce(() -> algaeRemover.resetPosition()));

    // algaeRemover manual down
    algaeDownButton
        .and(algaeModeButton)
        .whileTrue(algaeRemover.run(algaeRemover::algaeRemoverDown))
        .onFalse(algaeRemover.runOnce(algaeRemover::stopAlgaeRemover));
    // algaeRemover manual up
    algaeUpButton
        .and(algaeModeButton)
        .whileTrue(algaeRemover.run(algaeRemover::algaeRemoverUp))
        .onFalse(algaeRemover.runOnce(algaeRemover::stopAlgaeRemover));

    // algae intake sequence
    algaeIntakeSequenceButton
        .whileTrue(
            elevator
                .moveToSuppliedPosition(() -> drivetrain.getAlgaeHeight())
                .alongWith(
                    Commands.waitTime(AlgaeRemoverConstants.reefIntakeTimingOffset)
                        .andThen(
                            algaeRemover
                                .moveToPosition(AlgaeRemoverConstants.intakePosition)
                                .alongWith(algaeIntake.intake()))))
        .onFalse(
            algaeRemover
                .moveToPosition(AlgaeRemoverConstants.holdPosition)
                // .bringBackAlgaeRemover()
                .andThen(elevator.downPosition()));

    // barge score throw sequence
    bargeScoreSequenceButton
        .and(algaeModeButton.negate())
        .whileTrue(elevator.moveToPosition(ElevatorConstants.bargeHeight))
        .onFalse(
            Commands.sequence(
                algaeRemover
                    .moveToPosition(AlgaeRemoverConstants.bargeScorePosition)
                    .alongWith(
                        Commands.waitSeconds(.08)
                            .andThen(algaeIntake.outtake().withTimeout(0.458))),
                algaeRemover
                    .moveToPosition(AlgaeRemoverConstants.holdPosition)
                    // algaeRemover.bringBackAlgaeRemover()
                    .alongWith(elevator.downPosition())));

    // slow barge score
    bargeScoreSequenceButton
        .and(algaeModeButton)
        .whileTrue(
            Commands.sequence(
                elevator.moveToPosition(ElevatorConstants.bargeHeight),
                algaeRemover.moveToPosition(AlgaeRemoverConstants.bargeScorePosition)))
        .onFalse(
            Commands.sequence(
                algaeIntake.outtake().withTimeout(.9),
                algaeIntake.stop(),
                algaeRemover
                    .moveToPosition(AlgaeRemoverConstants.holdPosition)
                    .alongWith(elevator.downPosition())));

    //     algaeRemover
    //     .moveToPosition(AlgaeRemoverConstants.bargePosition)
    //     .alongWith(
    //         Commands.waitSeconds(.67).andThen(algaeIntake.outtake().withTimeout(.9))),
    // // algaeIntake.outtake().withTimeout(.9),
    // algaeIntake.stop(),
    // algaeRemover
    //     .moveToPosition(AlgaeRemoverConstants.holdPosition)
    //     .alongWith(elevator.downPosition())

    // score Processor

    processorScoreSequenceButton
        .and(algaeModeButton.negate())
        .whileTrue(
            elevator
                .downPosition()
                .alongWith(algaeRemover.moveToPosition(AlgaeRemoverConstants.processorPosition)))
        .onFalse(
            Commands.sequence(
                algaeIntake.outtake().withTimeout(1.0),
                algaeIntake.stop(),
                algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition)
                // algaeRemover.bringBackAlgaeRemover()
                ));

    // lollipop button
    processorScoreSequenceButton
        .and(algaeModeButton)
        .whileTrue(
            elevator
                .downPosition()
                .alongWith(
                    algaeRemover
                        .moveToPosition(AlgaeRemoverConstants.lollipopPosition)
                        .alongWith(algaeIntake.intake())))
        .onFalse(
            algaeRemover
                .moveToPosition(AlgaeRemoverConstants.holdPosition)
                .andThen(elevator.downPosition()));

    // algae floor intake
    algaeFloorButton
        .whileTrue(
            algaeRemover
                .moveToPosition(AlgaeRemoverConstants.floorAlgaePosition)
                .alongWith(algaeIntake.intake()))
        .onFalse(
            Commands.sequence(
                algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition),
                // algaeRemover.bringBackAlgaeRemover(),
                elevator.downPosition()));

    // manually outtake algae
    algaeManualOuttakeButton.whileTrue(algaeIntake.outtake()).onFalse(algaeIntake.stop());

    // manually intake algae
    algaeManualIntakeButton.whileTrue(algaeIntake.intake()).onFalse(algaeIntake.stop());

    // DOUBLE SCORE CODE
    doubleScoreMode.onFalse(
        Commands.runOnce(
            () -> {
              if (doubleScoreActive && selectedHeight.getAsDouble() > 0.0) {
                finishDoubleScoreCommand(selectedHeight).schedule();
              }
              doubleScoreActive = false;
              selectedHeight = () -> 0.0;
            }));

    // Height buttons trigger first phase while doubleScoreMode is active
    elevatorL2Button.onTrue(
        Commands.runOnce(
            () -> {
              if (doubleScoreActive) {
                selectedHeight = () -> ElevatorConstants.L2Height; // Example height
                startDoubleScoreCommand().schedule();
              }
            }));

    elevatorL3Button.onTrue(
        Commands.runOnce(
            () -> {
              if (doubleScoreActive) {
                selectedHeight = () -> ElevatorConstants.L3Height;
                startDoubleScoreCommand().schedule();
              }
            }));

    elevatorL4Button.onTrue(
        Commands.runOnce(
            () -> {
              if (doubleScoreActive) {
                selectedHeight = () -> ElevatorConstants.L4Height;
                startDoubleScoreCommand().schedule();
              }
            }));
  }

  //   private void temporaryAlgaeRemoverBindings() {
  //     algaeIntake.setDefaultCommand(algaeIntake.slowIntake());
  //     operatorController
  //         .leftBumper()
  //         .whileTrue(algaeRemover.run(algaeRemover::algaeRemoverUp))
  //         // .whileTrue(algaeRemover.moveToPosition(AlgaeRemoverConstants.bargePosition))
  //         .onFalse(algaeRemover.runOnce(algaeRemover::stopAlgaeRemover));
  //     operatorController
  //         .leftTrigger()
  //         .whileTrue(algaeRemover.run(algaeRemover::algaeRemoverDown))
  //         // .whileTrue(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition))
  //         .onFalse(algaeRemover.runOnce(algaeRemover::stopAlgaeRemover));
  //     operatorController
  //         .back()
  //         .and(operatorController.start().negate())
  //         .whileTrue(algaeIntake.intake())
  //         .onFalse(algaeIntake.stop());
  //     operatorController
  //         .start()
  //         .and(operatorController.back().negate())
  //         .and(operatorController.leftBumper())
  //         .whileTrue(algaeIntake.outtake())
  //         .onFalse(algaeIntake.stop());

  // algae reef intake sequence
  // operatorController
  //     .leftTrigger()
  //     .whileTrue(
  //         elevator
  //             .moveToSuppliedPosition(() -> drivetrain.getAlgaeHeight())
  //             .alongWith(
  //                 Commands.waitTime(AlgaeRemoverConstants.reefIntakeTimingOffset)
  //
  // .andThen(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition)))
  //             .alongWith(algaeIntake.intake()))
  //     .onFalse(
  //         algaeIntake
  //             .stop()
  //             .alongWith(
  //                 algaeRemover
  //                     .moveToPosition(AlgaeRemoverConstants.stowPosition)
  //                     .andThen(elevator.downPosition())));
  // }

  //   private void configureJoystickBindings() {
  //     //algae Remover bindings
  //     operatorStick
  //         .button(OperatorConstants.elevatorManualUp)
  //         .and(algaeMode)
  //         .whileTrue(algaeRemover.run(() -> algaeRemover.algaeRemoverUp()))
  //         .onFalse(algaeRemover.runOnce(() -> algaeRemover.stopAlgaeRemover()));
  //     operatorStick
  //         .button(OperatorConstants.elevatorManualDown)
  //         .and(algaeMode)
  //         .whileTrue(algaeRemover.run(() -> algaeRemover.algaeRemoverDown()))
  //         .onFalse(algaeRemover.runOnce(() -> algaeRemover.stopAlgaeRemover()));

  //         //elevator bindings
  //         elevator.setDefaultCommand(elevator.holdPosition());

  //     operatorStick
  //         .button(OperatorConstants.L4HeightButton)
  //         .and(algaeMode.negate().and(outtakeLaserBroken))
  //         .or(
  //             operatorStick
  //                 .button(OperatorConstants.elevatorOverrideButton)
  //                 .and(operatorStick.button(OperatorConstants.L4HeightButton))
  //                 .and(algaeMode.negate()))
  //         .onTrue(elevator.moveToPosition(ElevatorConstants.L4Height));

  //     operatorStick
  //         .button(OperatorConstants.L3HeightButton)
  //         .and(algaeMode.negate().and(outtakeLaserBroken))
  //         .or(
  //             operatorStick
  //                 .button(OperatorConstants.elevatorOverrideButton)
  //                 .and(operatorStick.button(OperatorConstants.L3HeightButton))
  //                 .and(algaeMode.negate()))
  //         .onTrue(elevator.moveToPosition(ElevatorConstants.L3Height));

  //     operatorStick
  //         .button(OperatorConstants.L2HeightButton)
  //         .and(algaeMode.negate().and(outtakeLaserBroken))
  //         .or(
  //             operatorStick
  //                 .button(OperatorConstants.elevatorOverrideButton)
  //                 .and(operatorStick.button(OperatorConstants.L2HeightButton))
  //                 .and(algaeMode.negate()))
  //         .onTrue(elevator.moveToPosition(ElevatorConstants.L2Height));

  //     operatorStick
  //         .button(OperatorConstants.elevatorDownButton)
  //         .and(algaeMode.negate())
  //         .onTrue(elevator.downPosition());

  //     operatorStick
  //         .button(OperatorConstants.homeElevatorButon)
  //         .and(algaeMode.negate())
  //         .onTrue(elevator.homeElevator());

  //     operatorStick
  //         .button(OperatorConstants.elevatorDownButton)
  //         .and(algaeMode)
  //         .onTrue(elevator.moveToPosition(ElevatorConstants.AlgaeLowHeight));

  //     operatorStick
  //         .button(OperatorConstants.L2HeightButton)
  //         .and(algaeMode)
  //         .onTrue(elevator.moveToPosition(ElevatorConstants.AlgaeHighHeight));

  //     operatorStick
  //         .button(OperatorConstants.coralInTheWay)
  //         .and(algaeMode.negate())
  //         .onTrue(
  //             new DeferredCommand(
  //                 () -> {
  //                   double newTarget =
  //                       Units.inchesToMeters(
  //                           elevator.getPositionInches() + ElevatorConstants.coralInTheWayAdd);
  //                   return elevator.moveToPosition(newTarget);
  //                 },
  //                 Set.of(elevator)));

  //     operatorStick
  //         .button(OperatorConstants.elevatorManualDown)
  //         .and(algaeMode.negate())
  //         .whileTrue(elevator.downSpeed(0.1))
  //         .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

  //     operatorStick
  //         .button(OperatorConstants.elevatorManualUp)
  //         .and(algaeMode.negate().and(outtakeLaserBroken))
  //         .or(
  //             operatorStick
  //                 .button(OperatorConstants.elevatorOverrideButton)
  //                 .and(operatorStick.button(OperatorConstants.elevatorManualUp))
  //                 .and(algaeMode.negate()))
  //         .whileTrue(elevator.upSpeed(0.1))
  //         .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

  //     operatorStick
  //         .button(OperatorConstants.algaeHighPosition)
  //         .and(algaeMode)
  //         .onTrue(
  //             elevator
  //                 .moveToPosition(ElevatorConstants.AlgaeHighHeight)
  //                 .alongWith(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition))
  //                 .alongWith(outtake.fastOuttake()));

  //     operatorStick
  //         .button(OperatorConstants.algaeLowPosition)
  //         .and(algaeMode)
  //         .onTrue(
  //             elevator
  //                 .moveToPosition(ElevatorConstants.AlgaeLowHeight)
  //                 .alongWith(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition))
  //                 .alongWith(outtake.fastOuttake()));
  // //indexer bindings
  // operatorStick
  // .button(OperatorConstants.indexerButton)
  // .and(algaeMode.negate())
  // .and(elevatorIsDown)
  // .whileTrue(indexer.runIndexer())
  // .onFalse(indexer.stop());

  // operatorStick
  // .button(OperatorConstants.indexerButton)
  // .and(algaeMode.negate())
  // .whileTrue(outtake.outtakeUntilBeamBreak())
  // .onFalse(outtake.stopOuttakeMotor());

  // operatorStick
  // .button(OperatorConstants.reverseIndexerButton)
  // .whileTrue(indexer.reverseIndexer())
  // .onFalse(indexer.stop());

  // //outtake bindings
  // operatorStick
  // .button(OperatorConstants.indexerButton)
  // .and(algaeMode)
  // .onTrue(outtake.reverseOuttake())
  // .onFalse(outtake.stopOuttakeMotor());

  // operatorStick
  // .button(OperatorConstants.outtakeButton)
  // .and(algaeMode.negate())
  // .onTrue(outtake.fastOuttake())
  // .onFalse(outtake.stopOuttakeMotor());
  //   }

  private void configureOperatorBindings() {
    configureAlgaeRemoverBindings();
    configureElevatorBindings();
    configureIndexerBindings();
    configureOuttakeBindings();
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
