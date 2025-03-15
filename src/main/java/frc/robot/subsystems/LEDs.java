// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import java.util.Map;
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

  private LEDPattern loadingPattern(LEDPattern base, int length, Time period) {
    final double periodMicros = period.in(Microseconds);

    return base.mapIndex(
        (bufLen, index) -> {
          final int margin = bufLen - length;

          double t = 2 * (RobotController.getTime() % periodMicros) / periodMicros;

          int start = (int) Math.round(Math.abs(margin * t - margin));

          return Math.floorMod(index + start, bufLen);
        });
  }

  public Command solidColor(Color c) {
    LEDPattern color = LEDPattern.solid(c);

    return run(
        () -> {
          color.applyTo(buffer);
        });
  }

  public Command loadingAnimation(Color color, int length, Time period) {
    LEDPattern base =
        (reader, writer) -> {
          int bufLen = reader.getLength();
          for (int i = 0; i < bufLen; i++) {
            if (i < length) {
              writer.setLED(i, color);
            } else {
              writer.setLED(i, Color.kBlack);
            }
          }
        };
    LEDPattern pattern = loadingPattern(base, length, period);
    LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    LEDPattern combination = rainbow.mask(pattern);

    return run(
        () -> {
          combination.applyTo(buffer);
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
    LEDPattern pattern =
        base.blink(Seconds.of(0.15)).atBrightness(Percent.of(75)).atBrightness(Percent.of(50));

    return run(
        () -> {
          pattern.applyTo(buffer);
        });
  }

  public Command breathe(Color color, Time period) {
    LEDPattern base = LEDPattern.solid(color);
    LEDPattern pattern = base.breathe(period);

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

  public Command scrolling(Color c1, Color c2) {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.4, Color.kBlack);
    LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, c1, c2);
    LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(30));

    LEDPattern pattern = base.mask(mask);

    return run(
        () -> {
          pattern.applyTo(buffer);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leds.setData(buffer);
  }
}
