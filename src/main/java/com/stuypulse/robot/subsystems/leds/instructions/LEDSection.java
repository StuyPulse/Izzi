/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

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

    public LEDSection(SLColor[] sections) {
        this.sections = sections;

        int totalLEDLength = Settings.LED.LED_LENGTH; // 15
        int sectionLength = totalLEDLength / sections.length; // 15 / 10 = 1
        int extraLEDS = totalLEDLength % sections.length; // 15 % 10 = 5

        this.separatorIndexes = new int[sections.length]; // int[10]

        // extra LEDs get distributed to the first few sections
        for (int i = 0; i < sections.length; i++) { // 0 - 9
            int offset = Math.min(extraLEDS, i); 
            int extraLEDForCurrentSection = (i < extraLEDS ? 1 : 0);
            this.separatorIndexes[i] = (i + 1) * sectionLength + offset + extraLEDForCurrentSection - 1;
            // (0 + 1) * 1 + 0 + 1 - 1 = 1
            // (1 + 1) * 1 + 1 + 1 - 1 = 3
            // (2 + 1) * 1 + 2 + 1 - 1 = 5
            // (3 + 1) * 1 + 3 + 1 - 1 = 7
            // (4 + 1) * 1 + 4 + 1 - 1 = 9
            // (5 + 1) * 1 + 5 + 0 - 1 = 10
            // (6 + 1) * 1 + 5 + 0 - 1 = 11
            // (7 + 1) * 1 + 5 + 0 - 1 = 12
            // (8 + 1) * 1 + 5 + 0 - 1 = 13
            // (9 + 1) * 1 + 5 + 0 - 1 = 14
        } 
    }

    @Override
    public void setLED(AddressableLEDBuffer ledBuffer) {
        int sectionIndex = 0;

        for (int ledIndex = 0; ledIndex < ledBuffer.getLength(); ledIndex++) { // 0 - 14

            ledBuffer.setLED(ledIndex, sections[sectionIndex].getColor8Bit());

            if (ledIndex == separatorIndexes[sectionIndex]) {
                sectionIndex++;
            }
        }
    }
}
