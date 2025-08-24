// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class IntakeAssistCommand extends Command {
  /** Creates a new TeleopSwerve. */
  private final Swerve swerve;

  private SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);

  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotationSupplier;

  private final Supplier<LinearVelocity> maxTranslationalSpeedSupplier;

  private final PIDController rotationController;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));
  private SlewRateLimiter rotationRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxAngularAcceleration.in(RotationsPerSecondPerSecond));

  public IntakeAssistCommand(
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      Supplier<LinearVelocity> maxTranslationalSpeed,
      Swerve swerve) {

    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.swerve = swerve;
    this.maxTranslationalSpeedSupplier = maxTranslationalSpeed;

    rotationController = new PIDController(7, 0, .5);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxTranslationalSpeed = maxTranslationalSpeedSupplier.get().in(MetersPerSecond);
    double forwardSpeed = -forwardSupplier.getAsDouble() * maxTranslationalSpeed;
    double strafeSpeed = -strafeSupplier.getAsDouble() * maxTranslationalSpeed;
    double rotationSpeed =
        -rotationSupplier.getAsDouble() * SwerveConstants.maxRotationalSpeed.in(RotationsPerSecond);

    forwardSpeed = forwardRateLimiter.calculate(forwardSpeed);
    strafeSpeed = strafeRateLimiter.calculate(strafeSpeed);
    rotationSpeed = rotationRateLimiter.calculate(rotationSpeed);

    if (Math.hypot(forwardSpeed, strafeSpeed) <= Units.inchesToMeters(0.5)) {
      forwardSpeed = 0;
      strafeSpeed = 0;

      forwardRateLimiter.reset(0);
      strafeRateLimiter.reset(0);
    }

    if (Math.abs(rotationSpeed) <= Units.degreesToRadians(1.5)) {
      rotationSpeed = 0;

      rotationRateLimiter.reset(0);
    }

    Pose2d robotPose = swerve.getState().Pose;

    List<Pose2d> targetPoses = swerve.getGamePiecePositions();
    // List<Pose2d> targetPoses =
    //     List.of(
    //         FieldConstants.testAlgaeRedLow,
    //         FieldConstants.testAlgaeBlueHigh,
    //         FieldConstants.testAlgaeRedHigh);

    if (targetPoses.size() == 0) {
      swerve.setControl(
          fieldOriented
              .withVelocityX(forwardSpeed)
              .withVelocityY(strafeSpeed)
              .withRotationalRate(Units.rotationsToRadians(rotationSpeed)));
      return;
    }

    Pose2d targetPose = robotPose.nearest(targetPoses);

    double distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());

    if (distanceToTarget > 5.0) {
      swerve.setControl(
          fieldOriented
              .withVelocityX(forwardSpeed)
              .withVelocityY(strafeSpeed)
              .withRotationalRate(Units.rotationsToRadians(rotationSpeed)));
      return;
    }

    Vector<N2> toTarget = new Vector<>(Nat.N2());

    if (AllianceUtil.isRedAlliance()) {
      // idk why but the vector is like reversed when we are in the other alliance and im too lazy
      // to actually think through the issue and this fix works :D
      toTarget.set(0, 0, robotPose.getX() - targetPose.getX());
      toTarget.set(1, 0, robotPose.getY() - targetPose.getY());

    } else {
      toTarget.set(0, 0, targetPose.getX() - robotPose.getX());
      toTarget.set(1, 0, targetPose.getY() - robotPose.getY());
    }

    Vector<N2> commandedVelocity = new Vector<>(Nat.N2());
    commandedVelocity.set(0, 0, forwardSpeed);
    commandedVelocity.set(1, 0, strafeSpeed);

    double dotProduct = commandedVelocity.unit().dot(toTarget.unit());

    boolean onTrack = dotProduct > 0.67;

    SmartDashboard.putNumber("Dot Product Value", dotProduct);
    SmartDashboard.putBoolean("DotProduct Prediction", onTrack);

    Vector<N2> projection = commandedVelocity.projection(toTarget);

    double projectionX = projection.get(0, 0);
    double projectionY = projection.get(1, 0);

    SmartDashboard.putNumber("Projection X", projectionX);
    SmartDashboard.putNumber("Projection Y", projectionY);
    SmartDashboard.putNumber("Forward Speed", forwardSpeed);
    SmartDashboard.putNumber("Strafe Speed", strafeSpeed);

    Rotation2d desiredRotation =
        targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle();

    double turningToTargetSpeed =
        rotationController.calculate(
            robotPose.getRotation().getRadians(), desiredRotation.getRadians());

    turningToTargetSpeed =
        MathUtil.clamp(
            turningToTargetSpeed,
            -SwerveConstants.maxRotationalSpeed.in(RadiansPerSecond),
            SwerveConstants.maxRotationalSpeed.in(RadiansPerSecond));

    if (onTrack) {
      swerve.setControl(
          fieldOriented
              .withVelocityX(projectionX)
              .withVelocityY(projectionY)
              .withRotationalRate(turningToTargetSpeed));

    } else {
      swerve.setControl(
          fieldOriented
              .withVelocityX(forwardSpeed)
              .withVelocityY(strafeSpeed)
              .withRotationalRate(Units.rotationsToRadians(rotationSpeed)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    forwardRateLimiter.reset(0);
    strafeRateLimiter.reset(0);
    rotationRateLimiter.reset(0);
    swerve.setControl(fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
