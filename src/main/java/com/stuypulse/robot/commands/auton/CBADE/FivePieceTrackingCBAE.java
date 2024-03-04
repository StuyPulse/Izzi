package com.stuypulse.robot.commands.auton.CBADE;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.auton.FollowPathTrackingAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathTrackingAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FivePieceTrackingCBAE extends SequentialCommandGroup {

    public FivePieceTrackingCBAE(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(-45)
            ),

            new ConveyorShootRoutine(),

            new FollowPathTrackingAndIntake(paths[0]),
            new SwerveDriveToShoot(2.9)
                .withTimeout(1.25),
            new ConveyorShootRoutine(),

            new FollowPathTrackingAndIntake(paths[1]),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            new FollowPathTrackingAndIntake(paths[2]),
            new SwerveDriveToShoot()//2.9)
                .withTolerance(0.05, 3),
            new ConveyorShootRoutine(),

            new FollowPathTrackingAndIntake(paths[3]),
            new FollowPathTrackingAlignAndShoot(paths[4], new SwerveDriveToShoot())
        );
    }
    
}
