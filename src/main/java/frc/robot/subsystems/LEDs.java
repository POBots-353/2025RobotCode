// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import java.util.function.DoubleSupplier;

public class LEDs extends SubsystemBase {
  private AddressableLED leds;
  private AddressableLEDBuffer buffer;

  /** Creates a new LED. */
  public LEDs() {
    leds = new AddressableLED(9);
    buffer = new AddressableLEDBuffer(38);
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  public Command solidColor(Color c) {
    LEDPattern color = LEDPattern.solid(c);

    return run(
        () -> {
          color.applyTo(buffer);
        });
  }

  public Command elevatorProgress(DoubleSupplier height) {
    LEDPattern base =
        LEDPattern.gradient(
            GradientType.kDiscontinuous, Color.kLightBlue, Color.kBlue, Color.kDarkBlue);
    LEDPattern pattern =
        LEDPattern.progressMaskLayer(() -> (height.getAsDouble() / ElevatorConstants.maxHeight));
    LEDPattern combination = base.mask(pattern);

    return run(
        () -> {
          combination.applyTo(buffer);
        });
  }

  public Command blink(Color c) {
    LEDPattern base = LEDPattern.solid(c);

    LEDPattern pattern = base.blink(Seconds.of(0.15)).atBrightness(Percent.of(75));

    return run(
        () -> {
          pattern.applyTo(buffer);
        });
  }

  public Command rainbowScroll() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Hertz.of(0.5));

    return run(
        () -> {
          rainbow.applyTo(buffer);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leds.setData(buffer);
  }
}
