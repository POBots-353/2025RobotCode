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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CoralAlign extends SequentialCommandGroup {
  /** Creates a new CoralAlign. */
  String Offset;

  Transform2d aprilTagOffset;

  Swerve drivetrain = TunerConstants.createDrivetrain();

  public CoralAlign(String Offset) {
    this.Offset = Offset;

    addCommands(pathfind());
  }

  public static Transform2d getBestTransform(List<Transform2d> transforms) {
    // Define a comparator to sort transforms by their magnitude
    Comparator<Transform2d> comparator =
        Comparator.comparingDouble(
            transform ->
                transform.getTranslation().getNorm()
                    + Math.abs(transform.getRotation().getRadians()));

    // Sort the list of transforms in ascending order of magnitude
    Collections.sort(transforms, comparator);

    // Return the first transform (which has the smallest magnitude)
    return transforms.get(0);
  }

  public Command pathfind() {
    List<PhotonPipelineResult> latestResults = drivetrain.latestLimelightResult;
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    List<Transform2d> transforms = new ArrayList<>();

    if (latestResults == null) {
      return new InstantCommand();
    }

    if (latestResults.isEmpty()) {
      return new InstantCommand();
    }

    for (PhotonPipelineResult result : latestResults) {
      if (!result.hasTargets()) {
        return new InstantCommand();
      } else {
        try {
          List<PhotonTrackedTarget> allTargets = result.getTargets();
          targets.addAll(allTargets);
          allTargets.clear();
        } catch (NullPointerException ex) {
          ex.printStackTrace();
          return new InstantCommand();
        }
      }
    }

    for (PhotonTrackedTarget target : targets) {
      transforms.add(
          new Transform2d(
              target.getBestCameraToTarget().getX(),
              target.getBestCameraToTarget().getY(),
              target.getBestCameraToTarget().getRotation().toRotation2d()));
    }

    Transform2d bestTransform = getBestTransform(transforms);

    if (Offset == "Left") {
      aprilTagOffset = new Transform2d(0, VisionConstants.aprilTagReefOffset, new Rotation2d(0));
    }
    if (Offset == "Right") {
      aprilTagOffset = new Transform2d(0, -VisionConstants.aprilTagReefOffset, new Rotation2d(0));
    }

    bestTransform = bestTransform.plus(aprilTagOffset);

    Pose2d aprilTagPose =
        drivetrain
            .getState()
            .Pose
            .transformBy(VisionConstants.limelightTransform2d)
            .transformBy(bestTransform);

    return AutoBuilder.pathfindToPose(
        aprilTagPose,
        new PathConstraints(3, 2, Units.degreesToRadians(540), Units.degreesToRadians(720)),
        0);
  }
}
