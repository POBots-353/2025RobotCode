// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class GroundIntakeS {

    @Override
  public Command Prematch(
    return Commands.sequence{
        Commands.runOnce(() -> {
              REVLibError motorError = groundIntake.getError();
              if (motorError != REVLibError.kOk) {
                addError("Error");
              } else {
                addInfo("Good");
              }
            }).
       
        Commands.wait(1);

        Commands.runOnce(() -> {   
            if (!speed()) {
            addError("Error");
          } else {
            addInfo("good");}
          }).

          Commands.runOnce(() -> {   
            if (!suction()) {
            addError("Error");
          } else {
            addInfo("good");}
          })
    }).
}           
       
        
          

