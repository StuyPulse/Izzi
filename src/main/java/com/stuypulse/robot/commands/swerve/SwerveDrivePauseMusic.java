package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDrivePauseMusic extends InstantCommand {
    
    private final SwerveDrive swerve;
    
    public SwerveDrivePauseMusic() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    }
    
    @Override
    public void execute() {
        swerve.pauseMusic();
    }

}