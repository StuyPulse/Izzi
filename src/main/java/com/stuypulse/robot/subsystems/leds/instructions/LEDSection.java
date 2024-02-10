package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSection implements LEDInstruction {

    public final SLColor[] sections;
    public final int[] separatorIndexes;

    public LEDSection(SLColor[] sections, int[] separatorIndexes) {
        if (sections.length - 1 != separatorIndexes.length) {
            throw new IllegalArgumentException("Mismatching section and seperating indexes");
        }

        this.sections = sections;
        this.separatorIndexes = separatorIndexes;
    }

    public LEDSection(SLColor[] sections){
        this.sections = sections;

        int totalLEDLength = Settings.LED.LED_LENGTH;
        int sectionLength = totalLEDLength / sections.length;
        int extraLEDS = totalLEDLength % sections.length;

        this.separatorIndexes = new int[sections.length];

        //extra LEDs get distributed to the first few sections
        for (int i = 0; i < sections.length; i++){
            int offset = Math.min(extraLEDS, i);
            int extraLEDForCurrentSection = (i < extraLEDS ? 1 : 0);
            this.separatorIndexes[i] = (i + 1) * sectionLength + offset + extraLEDForCurrentSection;
        }
    }

    @Override
    public void setLED(AddressableLEDBuffer ledBuffer){
        int sectionIndex = 0;
        
        for (int ledIndex = 0; ledIndex < ledBuffer.getLength(); ledIndex++) {
            if (sectionIndex > separatorIndexes[ledIndex]) {
                sectionIndex++;
            }

            ledBuffer.setLED(ledIndex, sections[sectionIndex].getColor8Bit());
        }
    }
}