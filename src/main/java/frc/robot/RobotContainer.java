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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToReef;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LogUtil;
import frc.robot.util.PersistentSendableChooser;

@Logged(strategy = Strategy.OPT_IN)
public class RobotContainer {
  @Logged(name = "Swerve")
  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  @Logged(name = "Elevator")
  private final Elevator elevator = new Elevator();

  @Logged(name = "Arm")
  private final Arm arm = new Arm();

  @Logged(name = "Indexer")
  private final Indexer indexer = new Indexer();

  @Logged(name = "Outtake")
  private final Outtake outtake = new Outtake();

  @Logged(name = "Ground Intake")
  private final GroundIntake groundIntake = new GroundIntake();

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

  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  private Trigger outtakeLaserBroken = new Trigger(outtake::outtakeLaserBroken);

  private Trigger buttonTrigger = new Trigger(elevator::buttonPressed);
  private Trigger elevatorIsDown = new Trigger(elevator::elevatorIsDown);
  private Trigger armMode = operatorStick.button(OperatorConstants.armModeButton);

  // Just put a bunch of instantcommands as placeholders for now
  Command outtakePrematch = new InstantCommand();
  Command algaeRemoverPrematch = new InstantCommand();
  Command armPrematch = new InstantCommand();
  Command elevatorPrematch = new InstantCommand();
  Command groundIntakePrematch = groundIntake.buildPrematch();
  Command indexerPrematch = indexer.buildPrematch();
  Command swervePrematch = new InstantCommand();

  public RobotContainer() {
    NamedCommands.registerCommand("Start Indexer", indexer.runIndexer().asProxy());
    NamedCommands.registerCommand("Stop Indexer", indexer.stop().asProxy());
    NamedCommands.registerCommand(
        "Elevator: L4",
        elevator
            .moveToPosition(ElevatorConstants.L4Height)
            // .onlyIf(outtakeLaserBroken)
            .withTimeout(4)
            .asProxy());
    NamedCommands.registerCommand("Auto Outtake", outtake.autoOuttake().withTimeout(3).asProxy());
    NamedCommands.registerCommand("Outtake", outtake.fastOuttake().withTimeout(1.5).asProxy());
    NamedCommands.registerCommand(
        "Elevator: Bottom", elevator.downPosition().withTimeout(4).asProxy());
    NamedCommands.registerCommand(
        "OuttakeUntilBeamBreak", outtake.outtakeUntilBeamBreak().withTimeout(5).asProxy());

    SmartDashboard.putData("Power Distribution", powerDistribution);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    drivetrain.configureAutoBuilder();

    configureDriverBindings();
    configureOperatorBindings();
    configureAutoChooser();
    configureBatteryChooser();

    // intakeLaserBroken
    //     .whileTrue(indexer.runIndexer())
    //     .onFalse(
    //         Commands.race(Commands.waitUntil(outtakeLaserBroken), Commands.waitSeconds(4))
    //             .andThen(indexer::stopIndexer));

    new Trigger(outtakeLaserBroken)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)),
                Commands.waitSeconds(2),
                Commands.runOnce(
                    () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0))));
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

    driverController.rightTrigger().whileTrue(drivetrain.humanPlayerAlign());

    driverController
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                drivetrain.pathFindToSetup(),
                new TurnToReef(drivetrain),
                Commands.waitSeconds(.08),
                drivetrain.reefAlign(true)));
    driverController
        .rightBumper()
        .whileTrue(
            Commands.sequence(
                drivetrain.pathFindToSetup(),
                new TurnToReef(drivetrain),
                Commands.waitSeconds(.08),
                drivetrain.reefAlign(false)));
    driverController.x().whileTrue(drivetrain.pathFindForAlgaeRemover());

    // reset the field-centric heading on left bumper press
    driverController
        .start()
        .and(driverController.back())
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);
    // driverController.triangle().onTrue(elevator.moveToPosition(ElevatorConstants.L4Height));
    // driverController.square().onTrue(elevator.moveToPosition(ElevatorConstants.L3Height));
    // driverController.circle().onTrue(elevator.moveToPosition(ElevatorConstants.L2Height));
    // driverController.cross().onTrue(elevator.downPosition());
  }

  private void configureElevatorBindings() {
    elevator.setDefaultCommand(elevator.holdPosition());

    // operatorStick
    //     .button(OperatorConstants.L4HeightButton)
    //     .and(armMode.negate())
    //     .onTrue(
    //         elevator
    //             .moveToPosition(ElevatorConstants.L4Height)
    //             .andThen(elevator.upSpeed(.1).withTimeout(.25)));

    operatorStick
        .button(OperatorConstants.L4HeightButton)
        .and(armMode.negate().and(outtakeLaserBroken))
        .or(
            operatorStick
                .button(OperatorConstants.elevatorOverrideButton)
                .and(operatorStick.button(OperatorConstants.L4HeightButton))
                .and(armMode.negate()))
        .onTrue(
            elevator
                .moveToPosition(ElevatorConstants.L4Height)
                .andThen(elevator.upSpeed(.1).withTimeout(.25)));

    operatorStick
        .button(OperatorConstants.L3HeightButton)
        .and(armMode.negate().and(outtakeLaserBroken))
        .or(
            operatorStick
                .button(OperatorConstants.elevatorOverrideButton)
                .and(operatorStick.button(OperatorConstants.L3HeightButton))
                .and(armMode.negate()))
        .onTrue(elevator.moveToPosition(ElevatorConstants.L3Height));

    // operatorStick
    //     .button(OperatorConstants.L2HeightButton)
    //     .and(armMode.negate())
    //     .and(outtakeLaserBroken)
    //     .onTrue(elevator.moveToPosition(ElevatorConstants.L2Height));

    operatorStick
        .button(OperatorConstants.L2HeightButton)
        .and(armMode.negate().and(outtakeLaserBroken))
        .or(
            operatorStick
                .button(OperatorConstants.elevatorOverrideButton)
                .and(operatorStick.button(OperatorConstants.L2HeightButton))
                .and(armMode.negate()))
        .onTrue(elevator.moveToPosition(ElevatorConstants.L2Height));

    operatorStick
        .button(OperatorConstants.elevatorDownButton)
        .and(armMode.negate())
        .onTrue(elevator.downPosition());

    operatorStick
        .button(OperatorConstants.homeElevatorButon)
        .and(armMode.negate())
        .onTrue(elevator.homeElevator());

    operatorStick
        .button(OperatorConstants.elevatorManualDown)
        .and(armMode.negate())
        .whileTrue(elevator.downSpeed(0.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

    operatorStick
        .button(OperatorConstants.elevatorManualUp)
        .and(armMode.negate())
        .whileTrue(elevator.upSpeed(0.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));

    operatorStick
        .button(OperatorConstants.elevatorManualUp)
        .and(armMode.negate().and(outtakeLaserBroken))
        .or(
            operatorStick
                .button(OperatorConstants.elevatorOverrideButton)
                .and(operatorStick.button(OperatorConstants.elevatorManualUp))
                .and(armMode.negate()))
        .whileTrue(elevator.upSpeed(0.1))
        .onFalse(elevator.runOnce(() -> elevator.stopElevator()));
  }

  private void configureArmBindings() {
    arm.setDefaultCommand(arm.moveToPosition(ArmConstants.armTopPosition));

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
    operatorStick
        .button(OperatorConstants.armPickupHeightButton)
        .and(armMode)
        .onTrue(arm.moveToPosition(ArmConstants.armBottomPosition));

    operatorStick
        .button(OperatorConstants.armL1HeightButton)
        .and(armMode)
        .onTrue(arm.moveToPosition(ArmConstants.armL1Position));
  }

  private void configureOuttakeBindings() {
    operatorStick
        .button(OperatorConstants.outtakeButton)
        .and(armMode.negate())
        .onTrue(outtake.fastOuttake())
        .onFalse(outtake.stopOuttakeMotor());
  }

  private void configureIndexerBindings() {
    operatorStick
        .button(OperatorConstants.indexerButton)
        .and(armMode.negate())
        .and(elevatorIsDown)
        .whileTrue(indexer.runIndexer())
        .onFalse(indexer.stop());

    operatorStick
        .button(OperatorConstants.indexerButton)
        .and(armMode.negate())
        .whileTrue(outtake.outtakeUntilBeamBreak())
        .onFalse(outtake.stopOuttakeMotor());

    operatorStick
        .button(OperatorConstants.reverseIndexerButton)
        .whileTrue(indexer.reverseIndexer())
        .onFalse(indexer.stop());
  }

  private void configureAlgaeRemoverBindings() {
    algaeRemover.setDefaultCommand(algaeRemover.moveToPosition(AlgaeRemoverConstants.downPosition));

    operatorStick
        .button(OperatorConstants.algaeRemoverHighPosition)
        .whileTrue(
            algaeRemover
                .moveToPosition(AlgaeRemoverConstants.horizontalPosition)
                .alongWith(outtake.fastOuttake())
                .alongWith(elevator.moveToPosition(ElevatorConstants.AlgaeHighHeight)));

    operatorStick
        .button(OperatorConstants.algaeRemoverLowPosition)
        .whileTrue(
            algaeRemover
                .moveToPosition(AlgaeRemoverConstants.horizontalPosition)
                .alongWith(outtake.fastOuttake())
                .alongWith(elevator.moveToPosition(ElevatorConstants.AlgaeLowHeight)));
  }

  private void configureOperatorBindings() {
    configureAlgaeRemoverBindings();
    configureArmBindings();
    configureElevatorBindings();
    configureIndexerBindings();
    configureOuttakeBindings();

    operatorStick
        .button(OperatorConstants.startingConfigButton)
        .whileTrue(
            elevator.downPosition().alongWith(arm.moveToPosition(ArmConstants.armTopPosition)))
        .onFalse(elevator.runOnce(elevator::stopElevator).alongWith(arm.runOnce(arm::stopArm)));
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
