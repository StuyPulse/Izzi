package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.subsystems.climber.Climber;

public class ClimberToBottom extends ClimberToHeight {

    private final Climber climber;

    public ClimberToBottom() {
        super(Settings.Climber.MIN_HEIGHT);
        climber = Climber.getInstance();

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        super.initialize();
        climber.setUsingBoth(false);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setUsingBoth(true);
    }
}
