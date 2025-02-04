// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  public static final int kPort = 9;
  public static final int kLength = 120;
  /** Creates a new LED. */

  public final AddressableLED Leds;
  public final AddressableLEDBuffer buffer;

  public LEDs() {

    Leds = new AddressableLED(kPort);
    buffer = new AddressableLEDBuffer(kLength);
    Leds.setLength(kLength);
    Leds.start();
    
    LEDPattern red = LEDPattern.solid(Color.kRed);

    red.applyTo(buffer);
    Leds.setData(buffer);

  }


  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));

  }

  
}


