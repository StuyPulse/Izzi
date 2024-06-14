package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChoreoSquare extends SequentialCommandGroup{
    public ChoreoSquare(String... paths) {
        PathPlannerPath Square = PathPlannerPath.fromChoreoTrajectory(paths[0].toString());

        addCommands(
            SwerveDrive.getInstance().followPathCommand(Square)
        );
    }
}
