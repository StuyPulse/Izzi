/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.stuylib.util.StopWatch;

import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RichieMode implements LEDInstruction {
    public SLColor color;
    private StopWatch stopwatch;
    private int index;
    private int prevIndex;

    public RichieMode(SLColor color) {
        this.color = color;
        stopwatch = new StopWatch();
    }

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        for (int i = 0; i < ledsBuffer.getLength(); i++) {
            ledsBuffer.setRGB(i, 0, 0, 0);
        }
        if (stopwatch.getTime() > 0.25) {
            ledsBuffer.setRGB(index, color.getRed(), color.getGreen(), color.getBlue());
            ledsBuffer.setRGB(prevIndex, 0, 0, 0);
            prevIndex = index;
            ++index;

            if (index == ledsBuffer.getLength()) {
                index = 0;
            }

            stopwatch.reset();
        }
    }
}
