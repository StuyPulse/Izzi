package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;

public class ClimberToTop extends ClimberToHeight {
    public ClimberToTop() {
        super(Settings.Climber.MAX_HEIGHT);
    }
}