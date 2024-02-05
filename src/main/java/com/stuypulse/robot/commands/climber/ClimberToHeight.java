package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberToHeight extends InstantCommand {

    public static Command untilDone(double height) {
        return new ClimberToHeight(height)
            .until(() -> Climber.getInstance().isAtTargetHeight(Settings.Climber.BangBang.THRESHOLD));
    }
    
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

}
