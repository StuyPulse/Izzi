package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.LEDColor;
import com.stuypulse.robot.constants.Settings;

public class LEDAutonChooser extends LEDSet {
    @Override
    public boolean runsWhenDisabled() {
        return Settings.LED.LED_AUTON_CHOOSE_TOGGLE.get();
    }

    public LEDAutonChooser() {
        super(LEDColor.AUTON_CHOOSER);
        
    }
}
