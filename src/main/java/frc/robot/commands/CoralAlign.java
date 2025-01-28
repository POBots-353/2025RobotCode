// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;


public class CoralAlign extends SequentialCommandGroup {
  /** Creates a new CoralAlign. */
  Transform2d aprilTagOffset;

  Swerve drivetrain = TunerConstants.createDrivetrain();

  public CoralAlign(String Offset) {

    drivetrain.configureAutoBuilder();

    if (Offset == "Left") {
      aprilTagOffset = new Transform2d(0, VisionConstants.aprilTagReefOffset, new Rotation2d(0));
    }
    if (Offset == "Right") {
      aprilTagOffset = new Transform2d(0, -VisionConstants.aprilTagReefOffset, new Rotation2d(0));
    }

    Transform2d transform = drivetrain.getBestAprilTag(drivetrain.latestLimelightResult);

    transform = transform.plus(aprilTagOffset);

    Pose2d aprilTagPose = drivetrain.getState().Pose.transformBy(transform);

    PathConstraints constraints =
        new PathConstraints(12.0, 10.0, Units.degreesToRadians(720), Units.degreesToRadians(720));

    Command pathfind = AutoBuilder.pathfindToPose(aprilTagPose, constraints, 0.0);

    addCommands(pathfind);
  }
}
