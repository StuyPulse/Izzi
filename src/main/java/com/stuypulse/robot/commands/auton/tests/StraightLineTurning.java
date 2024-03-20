package com.stuypulse.robot.commands.auton.tests;

import com.stuypulse.robot.commands.vision.VisionChangeWhiteList;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StraightLineTurning extends SequentialCommandGroup {
    
    public StraightLineTurning() {
        addCommands(
            new VisionChangeWhiteList(7, 8),
            SwerveDrive.getInstance().followPathCommand("Straight Line Turning"),
            new VisionReloadWhiteList()
        );
    }

}
