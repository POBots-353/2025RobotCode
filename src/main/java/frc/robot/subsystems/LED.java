// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class LED extends SubsystemBase {
  AddressableLED leds;
  AddressableLEDBuffer buffer;

  /** Creates a new LED. */
  public LED() {
    leds = new AddressableLED(9);
    buffer = new AddressableLEDBuffer(100);
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  public void setColor(Color c) {
    LEDPattern color = LEDPattern.solid(c);
    color.applyTo(buffer);
    leds.setData(buffer);
    leds.start();
  }

  public void elevatorLEDS(double d) {
    LEDPattern base =
        LEDPattern.gradient(
            GradientType.kDiscontinuous, Color.kLightBlue, Color.kBlue, Color.kDarkBlue);
    LEDPattern pattern =
        LEDPattern.progressMaskLayer(
            () -> (d / (Units.metersToInches(ElevatorConstants.maxHeight))));
    LEDPattern combination = base.mask(pattern);

    combination.applyTo(buffer);

    // Write the data to the LED strip
    leds.setData(buffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
