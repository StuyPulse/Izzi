package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Square extends SequentialCommandGroup {
    
    public Square() {
        addCommands(
            SwerveDrive.getInstance().followPathCommand("Square")
        );
    }

}
