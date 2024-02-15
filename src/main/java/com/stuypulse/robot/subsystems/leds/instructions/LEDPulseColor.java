/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.stuylib.util.StopWatch;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDPulseColor implements LEDInstruction {

    private final SLColor color;
    private final SLColor altcolor;
    private final StopWatch stopwatch;
    private final double pulseTime;

    public LEDPulseColor(SLColor color1, SLColor color2) {
        this(color1, color2, Settings.LED.BLINK_TIME.get());
    }

    public LEDPulseColor(SLColor color1, SLColor color2, double pulseTime) {
        this.color = color1;
        this.altcolor = color2;
        this.pulseTime = pulseTime;
        this.stopwatch = new StopWatch();
    }

    public LEDPulseColor(SLColor color, double pulseTime) {
        this(color, SLColor.BLACK, pulseTime);
    }

    public LEDPulseColor(SLColor color) {
        this(color, SLColor.BLACK, Settings.LED.BLINK_TIME.get());
    }

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        double time = stopwatch.getTime();

        if (time < pulseTime) {
            setColor(ledsBuffer, color);
        } else if (time < pulseTime * 2) {
            setColor(ledsBuffer, altcolor);
        } else {
            stopwatch.reset();
        }
    }

    private void setColor(AddressableLEDBuffer ledsBuffer, SLColor color) {
        for (int i = 0; i < ledsBuffer.getLength(); i++) {
            ledsBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        }
    }
}
