package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetOdometry;
import com.stuypulse.robot.commands.vision.VisionDisable;
import com.stuypulse.robot.commands.vision.VisionEnable;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Square extends SequentialCommandGroup {
    
    public Square() {
        addCommands(
            new SwerveDriveResetOdometry(PathPlannerPath.fromPathFile("SquareA").getPreviewStartingHolonomicPose()),
            new VisionDisable(),
            SwerveDrive.getInstance().followPathCommand("SquareA"),
            SwerveDrive.getInstance().followPathCommand("SquareB"),
            SwerveDrive.getInstance().followPathCommand("SquareC"),
            SwerveDrive.getInstance().followPathCommand("SquareD"),
            new VisionEnable()
        );
    }

}
