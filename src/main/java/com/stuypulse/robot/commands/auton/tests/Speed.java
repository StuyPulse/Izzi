package com.stuypulse.robot.commands.auton.tests;

import com.stuypulse.robot.commands.vision.VisionEnable;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Speed extends SequentialCommandGroup {
    
    public Speed() {
        addCommands(
            SwerveDrive.getInstance().followPathCommand("SPEED A"),
            SwerveDrive.getInstance().followPathCommand("SPEED B"),
            SwerveDrive.getInstance().followPathCommand("SPEED A"),
            SwerveDrive.getInstance().followPathCommand("SPEED B"),
            SwerveDrive.getInstance().followPathCommand("SPEED A"),
            SwerveDrive.getInstance().followPathCommand("SPEED B"),
            new VisionEnable()
        );
    }

}
