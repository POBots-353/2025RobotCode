package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class Light extends Command {
  public final Color color;
  public static LED leds;
  public Timer time = new Timer();

  public int period;
  private boolean lO = false;

  public Light(int period, Color color, LED leds) {
    this.period = (int) period;
    this.color = color;
    Light.leds = leds;

  }
  
  @Override
  public void initialize() {
    time.restart();
    time.start();
  }

  @Override
  public void execute() {
    if (time.advanceIfElapsed(period)) {
      AddressableLEDBuffer buffer = LED.updateBuffer();
      
      if (lO) {
        for (int e = 0; e < buffer.getLength(); e++) {
          buffer.setLED(e, color);
        }
      } else {
        for (int e = 0; e < buffer.getLength(); e++) {
          buffer.setLED(e, color);
        }
      }
    }

    }
  

  @Override
  public void end(boolean interrupted) {
    time.stop();
    time.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}