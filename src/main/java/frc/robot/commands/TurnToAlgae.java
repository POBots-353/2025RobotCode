// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurnToAlgae extends Command {
  /** Creates a new TeleopSwerve. */
  private final Swerve swerve;

  private SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);

  private SwerveRequest.RobotCentric robotOriented = new SwerveRequest.RobotCentric();
  // .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
  // .withSteerRequestType(SteerRequestType.Position);

  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private Debouncer rotationDebouncer = new Debouncer(2);

  private Rotation2d desiredRotation = Rotation2d.fromDegrees(5.1);

  private PIDController turnToAlgaeController =
      new PIDController(SwerveConstants.headingP, 0, SwerveConstants.headingD);

  private final Supplier<LinearVelocity> maxTranslationalSpeedSupplier;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));

  public TurnToAlgae(
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      Supplier<LinearVelocity> maxTranslationalSpeed,
      Swerve swerve) {

    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.swerve = swerve;
    this.maxTranslationalSpeedSupplier = maxTranslationalSpeed;

    turnToAlgaeController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private void updateDesiredRotation() {
    desiredRotation = swerve.desiredAlgaeRotation;
  }

  @Override
  public void execute() {
    updateDesiredRotation();
    double turningSpeed =
        turnToAlgaeController.calculate(
            swerve.getState().Pose.getRotation().getRadians(), desiredRotation.getRadians());

    double maxTranslationalSpeed = maxTranslationalSpeedSupplier.get().in(MetersPerSecond);
    double forwardSpeed = -forwardSupplier.getAsDouble() * maxTranslationalSpeed;
    double strafeSpeed = -strafeSupplier.getAsDouble() * maxTranslationalSpeed;

    forwardSpeed = forwardRateLimiter.calculate(forwardSpeed);
    strafeSpeed = strafeRateLimiter.calculate(strafeSpeed);

    if (Math.hypot(forwardSpeed, strafeSpeed) <= Units.inchesToMeters(0.5)) {
      forwardSpeed = 0;
      strafeSpeed = 0;

      forwardRateLimiter.reset(0);
      strafeRateLimiter.reset(0);
    }

    swerve.setControl(
        robotOriented
            .withVelocityX(forwardSpeed)
            .withVelocityY(strafeSpeed)
            .withRotationalRate(
                Units.rotationsToRadians(
                    turningSpeed
                    // * SwerveConstants.maxRotationalSpeed.in(RotationsPerSecond)
                    ))); // rotaitonspeed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    forwardRateLimiter.reset(0);
    strafeRateLimiter.reset(0);
    turnToAlgaeController.reset();
    swerve.setControl(robotOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(turnToAlgaeController.getError()) < Math.toRadians(3);
    return false;
  }
}
