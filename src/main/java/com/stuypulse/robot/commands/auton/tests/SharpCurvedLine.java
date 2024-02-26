package com.stuypulse.robot.commands.auton.tests;

import com.stuypulse.robot.commands.vision.VisionChangeWhiteList;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SharpCurvedLine extends SequentialCommandGroup {
    
    public SharpCurvedLine() {
        addCommands(
            new VisionChangeWhiteList(7, 8),
            SwerveDrive.getInstance().followPathCommand("Sharp Curved Line"),
            new VisionReloadWhiteList()
        );
    }

}
