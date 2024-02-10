package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSection implements LEDInstruction {
    public SLColor[] sections;
    public int[] seperatingIndexes;


    public LEDSection(SLColor[] sections, int[] seperatingIndexes) {
        if (sections.length - 1 != seperatingIndexes.length) {
            throw new IllegalArgumentException("Mismatching section and seperating indexes");
        }
        this.sections = sections;
        this.seperatingIndexes = seperatingIndexes;
    }

    public LEDSection(SLColor[] sections){
        this.sections = sections;

        int totalLEDLength = Settings.LED.LED_LENGTH;
        int sectionLength = totalLEDLength / sections.length;
        int extraLEDS = totalLEDLength % sections.length;

        this.seperatingIndexes = new int[sections.length];

        //extra LEDs get distributed to the first few sections
        for (int i = 0; i < sections.length; i++){
            int offset = Math.min(extraLEDS, i);
            int extraLEDForCurrentSection = (i < extraLEDS ? 1 : 0);
            this.seperatingIndexes[i] = (i + 1) * sectionLength + offset + extraLEDForCurrentSection;
        }
    }

    @Override
    public void setLED(AddressableLEDBuffer ledBuffer){
        int sectionIndex = 0;
        
        for (int ledIndex = 0; ledIndex < ledBuffer.getLength(); ledIndex++){
            if (seperatingIndexes[ledIndex] <= sectionIndex){
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