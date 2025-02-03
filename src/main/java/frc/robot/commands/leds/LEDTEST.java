package frc.robot.commands.leds;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/** Add your docs here. */
public class LEDTEST{

  public static void red(){
    
    try (AddressableLED led = new AddressableLED(6)) {
      AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(6);
      led.setLength(ledBuffer.getLength());

      led.setData(ledBuffer);
      led.start();
    @SuppressWarnings("unused")
    
   final int kPort = 9;
    final int kLength = 120;

    final AddressableLED m_led;
     final AddressableLEDBuffer m_buffer;

    public static void LEDSubsystem() {
      lOn = new AddressableLED(kPort);
      lOn = new AddressableLEDBuffer(kLength);
      m_led.setLength(kLength);
      m_led.start();

     
      setDefaultCommand(runPattern(LEDPattern.solid(Color.kGreenYellow)).withName("not on"));
    }

    private void setDefaultCommand(WrapperCommand withName) {
      throw new UnsupportedOperationException("??'");
    }

    @Override
    public void periodic() {
      // Periodically send the latest LED color data to the LED strip for it to display
      m_led.setData(m_buffer);
    }

    /**
     * 
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
      return LEDSubsystem(() -> pattern.applyTo(m_buffer)){}
    }
  
    
  }
}
}
    





