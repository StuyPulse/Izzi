package com.stuypulse.robot.commands.auton;

import java.util.Arrays;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.DoNothingCommand;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ChoosePath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FerryBottomAuton extends SequentialCommandGroup {
    public FerryBottomAuton(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(-45)
            ),

            new WaitCommand(Auton.SHOOTER_STARTUP_DELAY),

            new ConveyorShootRoutine(),


            new ChoosePath(
                new FollowPathAndIntake(paths[0]), 
                SwerveDrive.getInstance().followPathCommand(paths[1])              
            ),
            new FollowPathAndIntake(paths[1]),
            new ChoosePath(
                new FollowPathAndIntake(paths[2]), 
                SwerveDrive.getInstance().followPathCommand(paths[3])                
            ),
            new FollowPathAndIntake(paths[3]),
            new ChoosePath(
                new FollowPathAndIntake(paths[4]),
                SwerveDrive.getInstance().followPathCommand(paths[5])              
            ),
            new FollowPathAndIntake(paths[5])
        );
                        
    }
}
