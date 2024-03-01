package com.stuypulse.robot.commands.auton.CBADE;

import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BlayFivePieceCBAE extends SequentialCommandGroup {

    public BlayFivePieceCBAE() {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(-18)
            ),

            new ConveyorShootRoutine(),

            new FollowPathAndIntake("Blay First Piece To C"),
            new SwerveDriveToShoot(2.9),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("C to B"),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("B To A"),
            new SwerveDriveToShoot(2.9)
                .withTolerance(0.05, 3),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("A To E"),
            new FollowPathAlignAndShoot("E To Shoot", new SwerveDriveToShoot())
        );
    }
    
}