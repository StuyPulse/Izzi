/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds.instructions;

import java.util.Arrays;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SLColor;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSection implements LEDInstruction {

    private final SLColor[] sections;
    private final SLColor[] altSections;
    private final boolean isBlinking;
    private final int[] separatorIndexes;
    private final StopWatch stopWatch;

    public LEDSection(SLColor[] sections, SLColor[] altSections, int[] separatorIndexes, boolean isBlinking) {
        if (sections.length != separatorIndexes.length) {
            throw new IllegalArgumentException("Mismatching section and seperating indexes: \n Section Length (" + sections.length +") \n SeparatorIndexes ("+ separatorIndexes.length +")" );
        }

        this.sections = sections;
        this.altSections = altSections;
        this.separatorIndexes = separatorIndexes;
        this.isBlinking = isBlinking;
        this.stopWatch = new StopWatch();
    }

    public LEDSection(SLColor[] sections, SLColor[] altSections){
        this(sections, altSections, genSeparators(sections), true);
    }

    public LEDSection(SLColor[] sections, int[] separatorIndexes) {
        this(sections, genEmptyArray(sections.length), separatorIndexes, false);
    }

    public LEDSection(SLColor[] sections, boolean isBlinking) {
        this(sections, genEmptyArray(sections.length), genSeparators(sections), isBlinking);
    }

    @Override
    public void setLED(AddressableLEDBuffer ledBuffer) {
        if (isBlinking){
            double time = stopWatch.getTime();
            double pulseTime = Settings.LED.BLINK_TIME.get();

            if (time < pulseTime) {
                setColorSections(ledBuffer, this.sections, this.separatorIndexes);
            } else if (time < pulseTime * 2) {
                setColorSections(ledBuffer, this.altSections, this.separatorIndexes);
            } else {
                stopWatch.reset();
            }
        } else {
            setColorSections(ledBuffer, this.sections, this.separatorIndexes);
        }
    }

    private static SLColor[] genEmptyArray(int length) {
        SLColor[] array = new SLColor[length];
        Arrays.fill(array, SLColor.BLACK);
        return array;
    }

    private static int[] genSeparators(SLColor[] sections){
        int totalLEDLength = Settings.LED.LED_LENGTH;
        int sectionLength = totalLEDLength / sections.length;
        int extraLEDS = totalLEDLength % sections.length;

        int[] separators = new int[sections.length];

        // extra LEDs get distributed to the first few sections
        for (int i = 0; i < sections.length; i++) {
            int offset = Math.min(extraLEDS, i);
            int extraLEDForCurrentSection = (i < extraLEDS ? 1 : 0);
            separators[i] = (i + 1) * sectionLength + offset + extraLEDForCurrentSection - 1;
        }

        return separators;
    }

    private void setColorSections(AddressableLEDBuffer ledBuffer, SLColor[] sections, int[] separatorIndexes){
        int sectionIndex = 0;

        for (int ledIndex = 0; ledIndex < ledBuffer.getLength(); ledIndex++) {
            ledBuffer.setLED(ledIndex, sections[sectionIndex].getColor8Bit());

            if (ledIndex == separatorIndexes[sectionIndex]) {
                sectionIndex++;
            }
        }
    }
}
