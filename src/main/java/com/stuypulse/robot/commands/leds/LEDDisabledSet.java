package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;

public class LEDDisabledSet extends LEDSet{
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    public LEDDisabledSet(LEDInstruction ledInstruction) {
        super(ledInstruction);
    }
}
