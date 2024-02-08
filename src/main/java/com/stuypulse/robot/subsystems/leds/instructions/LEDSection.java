package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSection implements LEDInstruction {

    public SLColor[] sections;

    public LEDSection(SLColor[] sections) {
        this.sections = sections;
    }
        
    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) { 
        int sectionLength = ledsBuffer.getLength() / sections.length;
        int extraLEDs = ledsBuffer.getLength() % sections.length;

        int ledOffset = 0;

        for (int i = 0; i < sections.length; i++) {
            boolean appendLED = i + 1 <= extraLEDs;
            for (int j = 0; j < sectionLength + (appendLED ? 1 : 0); j++) {
                ledsBuffer.setRGB(i * sectionLength + ledOffset + j, sections[i].getRed(), sections[i].getGreen(), sections[i].getBlue());
            }
            if (appendLED) ledOffset++;
        }  
    }
}