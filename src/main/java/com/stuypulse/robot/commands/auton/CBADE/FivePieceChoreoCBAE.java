package com.stuypulse.robot.commands.auton.CBADE;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class FivePieceChoreoCBAE extends SequentialCommandGroup {



    public FivePieceChoreoCBAE(String... paths) {

        PathPlannerPath path1 = PathPlannerPath.fromChoreoTrajectory(paths[0].toString());
        PathPlannerPath path2 = PathPlannerPath.fromChoreoTrajectory(paths[1].toString());
        PathPlannerPath path3 = PathPlannerPath.fromChoreoTrajectory(paths[2].toString());
        PathPlannerPath path4 = PathPlannerPath.fromChoreoTrajectory(paths[3].toString());
        PathPlannerPath path5 = PathPlannerPath.fromChoreoTrajectory(paths[4].toString());


        
        addCommands(
            SwerveDrive.getInstance().followPathCommand(path1),
            SwerveDrive.getInstance().followPathCommand(path2),
            SwerveDrive.getInstance().followPathCommand(path3),
            SwerveDrive.getInstance().followPathCommand(path4),
            SwerveDrive.getInstance().followPathCommand(path5))
            ;

            
    }
}