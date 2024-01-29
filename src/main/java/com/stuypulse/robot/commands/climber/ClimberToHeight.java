package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberToHeight extends Command {
    
    private final Climber climber;
    private final double height;

    public ClimberToHeight(double height) {
        climber = Climber.getInstance();
        this.height = height;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setTargetHeight(height);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climber.getTargetHeight() - climber.getHeight()) < Settings.Climber.BangBang.THRESHOLD;
    }
}
