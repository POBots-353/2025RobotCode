// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Seconds;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.LEDPattern;
// import edu.wpi.first.wpilibj.LEDPattern.GradientType;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ElevatorConstants;

// public class LEDs extends SubsystemBase {
//   AddressableLED leds;
//   AddressableLEDBuffer buffer;

//   /** Creates a new LED. */
//   public LEDs() {
//     leds = new AddressableLED(8);
//     buffer = new AddressableLEDBuffer(15);
//     leds.setLength(buffer.getLength());
//     leds.setData(buffer);
//     leds.start();
//   }

//   public void setColor(Color c) {
//     LEDPattern color = LEDPattern.solid(c);
//     color.applyTo(buffer);
//     leds.setData(buffer);
//   }

//   public void elevatorLEDS(double d) {
//     LEDPattern base =
//         LEDPattern.gradient(
//             GradientType.kDiscontinuous, Color.kLightBlue, Color.kBlue, Color.kDarkBlue);
//     LEDPattern pattern =
//         LEDPattern.progressMaskLayer(
//             () -> (d / (Units.metersToInches(ElevatorConstants.maxHeight))));
//     LEDPattern combination = base.mask(pattern);

//     combination.applyTo(buffer);

//     // Write the data to the LED strip
//     leds.setData(buffer);
//   }

//   public void blinkyBlink(Color c) {
//     LEDPattern base = LEDPattern.solid(c);

//     // 1.5 seconds on, 1.5 seconds off, for a total period of 3 seconds
//     LEDPattern pattern = base.blink(Seconds.of(.353));

//     pattern.applyTo(buffer);
//     leds.setData(buffer);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
