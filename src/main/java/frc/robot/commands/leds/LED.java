// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorNew;

/** Add your docs here. */
public class LED extends Command {
    private static final boolean ElevatorNew = false;
        private final Color color;
        private final boolean lOn ;
        private Timer timer = new Timer();
                private LED leds;
                        public LED(double period, Color color, LED leds) {
                            this.color = color;
                            leds.leds = leds;
        }  
        @Override
        public void initialize() {
    
        
        public void Time(){
            timer.restart();
            timer.start();
        }
        public void Run(){
            AddressableLEDBuffer buffer = LED.updateBuffer();
            lOn = !lOn;
            if (lOn) {
              for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, color);
              }
            } else {
              for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, color);
              }
            }
        }
        public static AddressableLEDBuffer updateBuffer() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'updateBuffer'");
        }
}
        public static AddressableLEDBuffer updateBuffer() {
          // TODO Auto-generated method stub
          throw new UnsupportedOperationException("Unimplemented method 'updateBuffer'");
        }
}