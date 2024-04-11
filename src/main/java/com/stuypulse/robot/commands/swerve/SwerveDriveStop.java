package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveStop extends InstantCommand {
    
    public SwerveDriveStop() {
        super(() -> SwerveDrive.getInstance().stop());

        addRequirements(SwerveDrive.getInstance());
    }

}
