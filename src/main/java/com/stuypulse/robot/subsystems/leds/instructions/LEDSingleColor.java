/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDSingleColor implements LEDInstruction {

    private final SLColor color;

    public LEDSingleColor(SLColor color) {
        this.color = color;
    }

    public LEDSingleColor(Color color) {
        this(new SLColor(color));
    }

    public LEDSingleColor(Color8Bit color) {
        this(new SLColor(color));
    }

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        for (int i = 0; i < ledsBuffer.getLength(); i++) {
            ledsBuffer.setLED(i, color);
        }
    }
}
