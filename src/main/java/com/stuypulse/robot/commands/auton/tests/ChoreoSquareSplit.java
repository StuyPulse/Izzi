package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChoreoSquareSplit extends SequentialCommandGroup {
    public ChoreoSquareSplit(String... paths) {
        PathPlannerPath bottom = PathPlannerPath.fromChoreoTrajectory(paths[0].toString());
        PathPlannerPath right = PathPlannerPath.fromChoreoTrajectory(paths[1].toString());
        PathPlannerPath top = PathPlannerPath.fromChoreoTrajectory(paths[2].toString());
        PathPlannerPath left = PathPlannerPath.fromChoreoTrajectory(paths[3].toString());

        addCommands(
            SwerveDrive.getInstance().followPathCommand(bottom),
            SwerveDrive.getInstance().followPathCommand(right),
            SwerveDrive.getInstance().followPathCommand(top),
            SwerveDrive.getInstance().followPathCommand(left)
        );
    }
    
}