// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Elastic;
import frc.robot.util.ScoringSpot;
import java.util.Set;

public class AutoScoreCommand extends SequentialCommandGroup {

  Elastic.Notification fullNotification =
      new Elastic.Notification(
              Elastic.Notification.NotificationLevel.ERROR,
              "REEF FULL Notification",
              "No available scoring spots!")
          .withDisplaySeconds(3.53);

  // public AutoScoreCommand(Swerve swerve, Elevator elevator, Outtake outtake) {
  //     ScoringSpot bestSpot = ScoringSpot.getBestScoringSpot(swerve.getState().Pose);

  //     if (bestSpot != null) {
  //         addCommands(
  //             swerve.moveToPosition(bestSpot.position),
  //             elevator.moveToLevel(bestSpot.level),
  //             outtake.autoOuttake(),
  //             Commands.runOnce(()-> ScoringSpot.markSpotAsScored(bestSpot))

  //         );
  //     } else {
  //         addCommands(new InstantCommand(()-> Elastic.sendNotification(fullNotification)));
  //     }

  //     addRequirements(swerve, elevator, outtake);
  // }

  public AutoScoreCommand(Swerve swerve, Elevator elevator, Outtake outtake) {
    addCommands(
        new DeferredCommand(
            () -> {
              ScoringSpot bestSpot = ScoringSpot.getBestScoringSpot(swerve.getState().Pose);
              if (bestSpot != null) {
                return new SequentialCommandGroup(
                    swerve.moveToPosition(bestSpot.position),
                    elevator.moveToLevel(bestSpot.level),
                    outtake.autoOuttake(),
                    Commands.runOnce(() -> ScoringSpot.markSpotAsScored(bestSpot)));
              } else {
                return new InstantCommand(() -> Elastic.sendNotification(fullNotification));
              }
            },
            Set.of(swerve, elevator, outtake)));
  }
}
