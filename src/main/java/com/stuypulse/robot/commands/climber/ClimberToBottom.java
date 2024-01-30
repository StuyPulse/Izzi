package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;

public class ClimberToBottom extends ClimberToHeight {
    public ClimberToBottom() {
        super(Settings.Climber.MIN_HEIGHT);
    }
}
