package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChoreoStraightLine extends SequentialCommandGroup{
    public ChoreoStraightLine(String... paths){
        PathPlannerPath Straight = PathPlannerPath.fromChoreoTrajectory(paths[0].toString());

        addCommands(SwerveDrive.getInstance().followPathCommand(Straight));
    }

    
}
