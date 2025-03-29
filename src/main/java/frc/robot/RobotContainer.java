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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LogUtil;
import frc.robot.util.PersistentSendableChooser;
import java.util.Set;
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
  private boolean positionMode = false;
  private boolean doubleScoreActive = false;
  private DoubleSupplier selectedHeight = () -> 0.0;
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private Debouncer loopDebouncer = new Debouncer(2);

  private Trigger outtakeLaserBroken = new Trigger(outtake::outtakeLaserBroken);
  private Trigger isPositionMode = new Trigger(() -> positionMode);
  private Trigger buttonTrigger = new Trigger(elevator::buttonPressed);
  private Trigger elevatorIsDown = new Trigger(elevator::elevatorIsDown);
  //   private Trigger algaeMode = operatorStick.button(OperatorConstants.algaeModeButton);
  private Trigger atValidReefPose = new Trigger(drivetrain::atValidReefPose);
  private Trigger atElevatorHeight = new Trigger(elevator::atSetHeight);
  private Trigger doubleScoreMode =
      operatorController.leftBumper().and(operatorController.start().negate());

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

    NamedCommands.registerCommand("Turn to reef", new TurnToReef(drivetrain).withTimeout(2));
    NamedCommands.registerCommand("AutoAlignLeft", drivetrain.reefAlign(true).withTimeout(2.8));
    NamedCommands.registerCommand("AutoAlignRight", drivetrain.reefAlign(false).withTimeout(2.8));
    NamedCommands.registerCommand(
        "Double Score: 1", startDoubleScoreCommand().withTimeout(1.0).asProxy());
    NamedCommands.registerCommand(
        "Double Score: 2",
        finishDoubleScoreCommand(() -> ElevatorConstants.L4Height).withTimeout(3).asProxy());
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
                                    .moveToPosition(AlgaeRemoverConstants.intakePosition)
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
            .alongWith(Commands.waitSeconds(.08).andThen(algaeIntake.outtake()))
            .withTimeout(1.3)
            .asProxy());

    NamedCommands.registerCommand(
        "Barge Score: 3",
        algaeRemover
            .moveToPosition(AlgaeRemoverConstants.holdPosition)
            .alongWith(elevator.downPosition())
            .withTimeout(1.1)
            .asProxy());
    NamedCommands.registerCommand(
        "Algae Remover: STOW",
        algaeRemover.moveToPosition(AlgaeRemoverConstants.stowPosition).withTimeout(2).asProxy());

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
    new Trigger(DriverStation::isDSAttached).onChange(led.runOnce(() -> {}).ignoringDisable(true));

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

    driverController
        .leftStick()
        .whileTrue(
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

    driverController
        .y()
        .onTrue(
            new DeferredCommand(
                    () -> autoChooser.getSelected(),
                    Set.of(drivetrain, elevator, indexer, algaeIntake, algaeRemover, outtake))
                .asProxy());

    driverController
        .rightStick()
        .whileTrue(
            new TurnToAlgae(
                driverController::getLeftY,
                driverController::getLeftX,
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

    // driverController.leftStick().whileTrue(drivetrain.pathFindToBarge());

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

    driverController
        .leftBumper()
        .and(isPositionMode.negate())
        .whileTrue(
            Commands.sequence(
                drivetrain.pathFindToSetup(),
                new TurnToReef(drivetrain),
                drivetrain.reefAlign(true)));
    // drivetrain.reefAlign(true),
    // Commands.waitUntil(
    //     outtakeLaserBroken
    //         .and(doubleScoreMode.negate())
    //         // .and(atValidReefPose)
    //         .and(elevatorIsDown.negate())
    //         .and(atElevatorHeight)),
    // outtake.autoOuttake()));

    driverController
        .rightBumper()
        .and(isPositionMode.negate())
        .whileTrue(
            Commands.sequence(
                drivetrain.pathFindToSetup(),
                new TurnToReef(drivetrain),
                drivetrain.reefAlign(false)));
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
    elevator.setDefaultCommand(
        Commands.either(Commands.none(), elevator.holdPosition(), buttonTrigger));

    // Elevator L4
    operatorController
        .b()
        .and(outtakeLaserBroken)
        .and(operatorController.leftBumper().negate())
        .or(operatorController.povLeft().and(operatorController.b()))
        .onTrue(
            elevator
                .moveToPosition(ElevatorConstants.L4Height)
                .andThen(
                    new ConditionalCommand(
                        outtake.autoOuttake(),
                        Commands.none(),
                        () ->
                            (driverController.leftBumper().getAsBoolean()
                                    || driverController.rightBumper().getAsBoolean())
                                && outtakeLaserBroken.getAsBoolean())));

    // elevator L3
    operatorController
        .y()
        .and(outtakeLaserBroken)
        .and(operatorController.leftBumper().negate())
        .or(operatorController.povLeft().and(operatorController.y()))
        .onTrue(
            elevator
                .moveToPosition(ElevatorConstants.L3Height)
                .andThen(
                    new ConditionalCommand(
                        outtake.autoOuttake(),
                        Commands.none(),
                        () ->
                            (driverController.leftBumper().getAsBoolean()
                                    || driverController.rightBumper().getAsBoolean())
                                && outtakeLaserBroken.getAsBoolean())));

    // elevator L2
    operatorController
        .x()
        .and(outtakeLaserBroken)
        .and(operatorController.leftBumper().negate())
        .or(operatorController.povLeft().and(operatorController.x()))
        .onTrue(
            elevator
                .moveToPosition(ElevatorConstants.L2Height)
                .andThen(
                    new ConditionalCommand(
                        outtake.autoOuttake(),
                        Commands.none(),
                        () ->
                            (driverController.leftBumper().getAsBoolean()
                                    || driverController.rightBumper().getAsBoolean())
                                && outtakeLaserBroken.getAsBoolean())));

    // elevator down height
    operatorController.a().onTrue(elevator.downPosition());

    // home elevator
    operatorController.back().and(operatorController.start()).onTrue(elevator.homeElevator());

    // coral in the way add
    operatorController
        .povRight()
        .onTrue(
            new DeferredCommand(
                () -> {
                  double newTarget =
                      Units.inchesToMeters(
                          elevator.getPositionInches() + ElevatorConstants.coralInTheWayAdd);
                  return elevator.moveToPosition(newTarget);
                },
                Set.of(elevator)));
    // elevator manual down
    operatorController
        .povDown()
        .and(operatorController.leftBumper().negate())
        .whileTrue(elevator.downSpeed(0.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));
    // elevator manual up
    operatorController
        .povUp()
        .and(operatorController.leftBumper().negate())
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

    operatorController
        .start()
        .and(operatorController.back().negate())
        .and(operatorController.leftBumper().negate())
        .whileTrue(outtake.fastOuttake())
        .onFalse(outtake.stopOuttakeMotor());
  }

  private void configureIndexerBindings() {
    operatorController
        .rightTrigger()
        .and(elevatorIsDown)
        .and(operatorController.leftBumper().negate())
        .whileTrue(indexer.runIndexer().alongWith(outtake.outtakeUntilBeamBreak()))
        .onFalse(indexer.stop().alongWith(outtake.stopOuttakeMotor()));

    operatorController
        .back()
        .and(operatorController.start().negate())
        .whileTrue(indexer.reverseIndexer())
        .onFalse(indexer.stop());
  }

  private void configureAlgaeRemoverBindings() {
    algaeIntake.setDefaultCommand(
        Commands.either(
            Commands.none(),
            algaeIntake.slowIntake(),
            () -> DriverStation.isAutonomous() && !isAlgaeAuto));

    // algaeintake to stow position
    operatorController
        .back()
        .and(operatorController.start())
        .onTrue(algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition));

    // algaeRemover manual down
    operatorController
        .povUp()
        .and(operatorController.leftBumper())
        .whileTrue(algaeRemover.run(algaeRemover::algaeRemoverDown))
        .onFalse(algaeRemover.runOnce(algaeRemover::stopAlgaeRemover));
    // algaeRemover manual up
    operatorController
        .povDown()
        .and(operatorController.leftBumper())
        .whileTrue(algaeRemover.run(algaeRemover::algaeRemoverUp))
        .onFalse(algaeRemover.runOnce(algaeRemover::stopAlgaeRemover));

    // algae intake sequence
    operatorController
        .leftTrigger()
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
                .andThen(elevator.downPosition()));

    // barge score sequence
    operatorController
        .leftStick()
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
    operatorController
        .rightStick()
        .whileTrue(
            elevator
                .downPosition()
                .alongWith(algaeRemover.moveToPosition(AlgaeRemoverConstants.processorPosition)))
        .onFalse(
            Commands.sequence(
                algaeIntake.outtake().withTimeout(1.0),
                algaeIntake.stop(),
                algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition)));

    // algae floor intake
    operatorController
        .rightBumper()
        .whileTrue(
            algaeRemover
                .moveToPosition(AlgaeRemoverConstants.floorAlgaePosition)
                .alongWith(algaeIntake.intake()))
        .onFalse(
            Commands.sequence(
                algaeRemover.moveToPosition(AlgaeRemoverConstants.holdPosition),
                elevator.downPosition()));

    // manually outtake algae
    operatorController
        .start()
        .and(operatorController.back().negate())
        .and(operatorController.leftBumper())
        .whileTrue(algaeIntake.outtake())
        .onFalse(algaeIntake.stop());

    // manually intake algae
    operatorController
        .rightTrigger()
        .and(operatorController.leftBumper())
        .whileTrue(algaeIntake.intake())
        .onFalse(algaeIntake.stop());

    // score algae + coral MODE
    // operatorController
    //     .leftStick()
    //     .and(operatorController.rightStick())
    //     .whileTrue(
    //         Commands.sequence(
    //             elevator.moveToPosition(drivetrain.getAlgaeHeight()),
    //             algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition),
    //             algaeRemover.intake()))
    //     .onFalse(
    //         Commands.sequence(
    //             algaeRemover.stop(),
    //             algaeRemover.moveToPosition(AlgaeRemoverConstants.stowPosition),
    //             elevator.moveToPosition(ElevatorConstants.L4Height),
    //             outtake.autoOuttake(),
    //             elevator.downPosition()));

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
    operatorController
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (doubleScoreActive) {
                    selectedHeight = () -> ElevatorConstants.L2Height; // Example height
                    startDoubleScoreCommand().schedule();
                  }
                }));

    operatorController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (doubleScoreActive) {
                    selectedHeight = () -> ElevatorConstants.L3Height;
                    startDoubleScoreCommand().schedule();
                  }
                }));

    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (doubleScoreActive) {
                    selectedHeight = () -> ElevatorConstants.L4Height;
                    startDoubleScoreCommand().schedule();
                  }
                }));
  }

  private void temporaryAlgaeRemoverBindings() {
    algaeIntake.setDefaultCommand(algaeIntake.slowIntake());
    operatorController
        .leftBumper()
        .whileTrue(algaeRemover.run(algaeRemover::algaeRemoverUp))
        // .whileTrue(algaeRemover.moveToPosition(AlgaeRemoverConstants.bargePosition))
        .onFalse(algaeRemover.runOnce(algaeRemover::stopAlgaeRemover));
    operatorController
        .leftTrigger()
        .whileTrue(algaeRemover.run(algaeRemover::algaeRemoverDown))
        // .whileTrue(algaeRemover.moveToPosition(AlgaeRemoverConstants.intakePosition))
        .onFalse(algaeRemover.runOnce(algaeRemover::stopAlgaeRemover));
    operatorController
        .back()
        .and(operatorController.start().negate())
        .whileTrue(algaeIntake.intake())
        .onFalse(algaeIntake.stop());
    operatorController
        .start()
        .and(operatorController.back().negate())
        .and(operatorController.leftBumper())
        .whileTrue(algaeIntake.outtake())
        .onFalse(algaeIntake.stop());

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
  }

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
