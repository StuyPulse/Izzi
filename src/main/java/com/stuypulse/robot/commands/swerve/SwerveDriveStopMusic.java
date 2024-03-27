package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveStopMusic extends InstantCommand {
    
    private final SwerveDrive swerve;
    
    public SwerveDriveStopMusic() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    }
    
    @Override
    public void execute() {
        swerve.stopMusic();
    }

}