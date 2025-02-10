// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class RSLSync extends Command {
  private final Color color;
  private boolean ledsOn = false;

  /** Creates a new RSLSync. */
  public RSLSync(Color color, LED leds) {
    this.color = color;
  }


  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean rslOn = RobotController.getRSLState();
    AddressableLEDBuffer buffer = LED.updateBuffer();

    if (rslOn) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
      LED.updateBuffer();
    } else if (!rslOn) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    }

    ledsOn = rslOn;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean rslOn = RobotController.getRSLState();
    AddressableLEDBuffer buffer = LED.updateBuffer();

    if (rslOn && !ledsOn) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    } else if (!rslOn && ledsOn) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    }

    ledsOn = rslOn;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
