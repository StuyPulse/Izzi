package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Mobility extends SequentialCommandGroup {
    
    public Mobility() {
        addCommands(
            SwerveDrive.getInstance().followPathCommand("Mobility")
        );
    }

}
