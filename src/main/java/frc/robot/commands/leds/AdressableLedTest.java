// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;

/** Add your docs here. */
public class AdressableLedTest {
    m_led = new AddressableLED(9);

    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
    AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(120);

    AddressableLEDBufferView m_left = m_buffer.createView(0, 59);


    AddressableLEDBufferView m_right = m_buffer.createView(60, 119).reversed();
}

