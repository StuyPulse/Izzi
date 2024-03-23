package com.stuypulse.robot.commands.auton.CBADE;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.conveyor.ConveyorToShooter;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Shoots no shots at podium
 */
public class FivePieceCBAE extends SequentialCommandGroup {

    public FivePieceCBAE(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(-18)
            ),

            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[0]),

            new FollowPathAlignAndShoot(paths[1], new SwerveDriveToShoot()),

            new FollowPathAndIntake(paths[2]),
            new SwerveDriveToShoot()
                .alongWith(new ConveyorToShooter()),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[3]),
            new SwerveDriveToShoot()
                .alongWith(new ConveyorToShooter()),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[4]),
            new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot())
        );
    }
    
}
