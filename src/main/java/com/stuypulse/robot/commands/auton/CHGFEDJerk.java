package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CHGFEDJerk extends SequentialCommandGroup {
    public CHGFEDJerk(PathPlannerPath... paths) {
        addCommands(
            //shooting preload
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(-45)
                    .withTolerance(0.1, 0.1, 2)
            ),

            new ConveyorShootRoutine(),

            //start to H & H to HFerry
            new FollowPathAndIntake(paths[0]),
            new FollowPathAndIntake(paths[1]),

            //shooting 
            new ConveyorShootRoutine(),

            //HFerry to G
            new FollowPathAndIntake(paths[2]),
    
            //G to GFerry
            new FollowPathAndIntake(paths[3]),

            //shooting
            new ConveyorShootRoutine(),

            //GFerry to F
            new FollowPathAndIntake(paths[4]),

            //F to FFerry
            new FollowPathAndIntake(paths[5]),

            //shooting
            new ConveyorShootRoutine(),

            //FFerry to E
            new FollowPathAndIntake(paths[6]),

            //E to EFerry
            new FollowPathAndIntake(paths[7]),

            //shooting
            new ConveyorShootRoutine(),

            //EFerry to D
            new FollowPathAndIntake(paths[8])
        );
    }
    
}
