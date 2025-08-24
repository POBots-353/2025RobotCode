package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Elastic;
import java.util.Set;

public class AutoPickUp extends SequentialCommandGroup {
  Elastic.Notification noPiecesNotification =
      new Elastic.Notification(
              Elastic.Notification.NotificationLevel.ERROR,
              "No Game Pieces",
              "No Trackable Game Pieces!")
          .withDisplaySeconds(3.53);

  public AutoPickUp(Swerve swerve, Indexer intake, Elevator arm) {
    addCommands(
        new DeferredCommand(
            () -> {
              if (!swerve.getGamePiecePositions().isEmpty()) {

                Pose2d robotPose = swerve.getState().Pose;
                Pose2d closestPiece = robotPose.nearest(swerve.getGamePiecePositions());

                Translation2d toPiece =
                    closestPiece.getTranslation().minus(robotPose.getTranslation());

                Translation2d direction = toPiece.div(toPiece.getNorm());

                Translation2d targetTranslation =
                    closestPiece.getTranslation().minus(direction.times(.3048));

                Pose2d targetPose = new Pose2d(targetTranslation, toPiece.getAngle());

                return new ParallelDeadlineGroup(
                    swerve.moveToPosition(targetPose),
                    arm.downPosition().withTimeout(2),
                    intake.runIndexer().withTimeout(1));
              } else {
                return new InstantCommand(() -> Elastic.sendNotification(noPiecesNotification));
              }
            },
            Set.of(swerve, intake, arm)));
  }
}
