package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.subsystems.climber.*;

public class ClimberToTop extends ClimberToHeight {
    
    private final Climber climber;
       
    public ClimberToTop() {
        super(Settings.Climber.MAX_HEIGHT);
        climber = Climber.getInstance();

        addRequirements(climber);
    }

    public void initialize() {
        super.initialize();
        climber.setUsingBoth(true);
    }
    
}