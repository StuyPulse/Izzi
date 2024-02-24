package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.vision.VisionChangeWhiteList;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Square extends SequentialCommandGroup {
    
    public Square() {
        addCommands(
            new VisionChangeWhiteList(7, 8),
            SwerveDrive.getInstance().followPathCommand("Square"),
            new VisionReloadWhiteList()
        );
    }

}
