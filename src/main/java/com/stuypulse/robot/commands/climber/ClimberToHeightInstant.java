package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberToHeightInstant extends InstantCommand {

    private final Climber climber;
    private final double height;

    public ClimberToHeightInstant(double height) {
        climber = Climber.getInstance();
        this.height = height;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setTargetHeight(height);
    }
}
