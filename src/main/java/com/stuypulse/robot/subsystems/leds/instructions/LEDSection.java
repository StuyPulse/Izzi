package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSection implements LEDInstruction {
    public SLColor[] sections;
    public int[] indexes;


    public LEDSection(SLColor[] sections, int[] indexes) {
        if (sections.length != indexes.length) {
            throw new IllegalArgumentException("Mismatching section and length indexes");
        }
        this.sections = sections;
        this.indexes = indexes;
    }

    public LEDSection(SLColor[] sections){
        this.sections = sections;

        int totalLEDLength = Settings.LED.LED_LENGTH;
        int sectionLength = totalLEDLength / sections.length;
        int extraLEDS = totalLEDLength % sections.length;

        this.indexes = new int[sections.length];

        //extra LEDs get distributed to the first few sections
        for (int i = 0; i < sections.length; i++){
            int offset = Math.min(extraLEDS, i);
            int extraLEDForCurrentSection = (i < extraLEDS ? 1 : 0);
            this.indexes[i] = (i + 1) * sectionLength + offset + extraLEDForCurrentSection;
        }
    }

    @Override
    public void setLED(AddressableLEDBuffer ledBuffer){
        int sectionIndex = 0;
        
        for (int ledIndex = 0; ledIndex < ledBuffer.getLength(); ledIndex++){
            if (indexes[sectionIndex] <= ledIndex){
                sectionIndex++;
            }

            SLColor color = sections[sectionIndex];
            ledBuffer.setRGB(ledIndex,
                color.getRed(),
                color.getGreen(),
                color.getBlue()
            );
        }
    }
}