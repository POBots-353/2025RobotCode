// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;

public class TurnToReef extends Command {
  private final Swerve swerve;
  private double targetX, targetY;
  private Pose2d target;
  private final PIDController rotationController;
  private SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);

  public TurnToReef(Swerve swerve) {
    this.swerve = swerve;
    rotationController = new PIDController(10, 0, 0); // kP, kI, kD
    rotationController.enableContinuousInput(-Math.PI, Math.PI); // Allow wrapping at -π to π

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    target = AllianceUtil.getReefPose();
    targetX = target.getX();
    targetY = target.getY();
  }

  @Override
  public void execute() {

    Pose2d currentPose = swerve.getState().Pose;

    double deltaX = targetX - currentPose.getX();
    double deltaY = targetY - currentPose.getY();
    double targetAngle = Math.atan2(deltaY, deltaX);

    double currentAngle = currentPose.getRotation().getRadians();

    double rotationSpeed = rotationController.calculate(currentAngle, targetAngle);

    rotationSpeed =
        MathUtil.clamp(
            rotationSpeed,
            -SwerveConstants.maxRotationalSpeed.in(RadiansPerSecond),
            SwerveConstants.maxRotationalSpeed.in(RadiansPerSecond));

    swerve.setControl(
        fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(rotationSpeed));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(rotationController.getPositionError()) < Math.toRadians(3);
  }

  @Override
  public void end(boolean interrupted) {
    rotationController.reset();
    swerve.setControl(fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }
}
