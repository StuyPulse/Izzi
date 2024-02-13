package com.stuypulse.robot.subsystems.leds.instructions;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.stuypulse.robot.util.SLColor;

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
