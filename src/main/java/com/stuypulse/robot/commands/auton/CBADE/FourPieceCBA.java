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

public class FourPieceCBA extends SequentialCommandGroup {

    public FourPieceCBA() {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(-45)
            ),

            new ConveyorShootRoutine(),

            // new FollowPathAndIntake("First Piece to C"),
            // new SwerveDriveToShoot(2.9),
            // new ConveyorShootRoutine(),

            // new FollowPathAndIntake("C to B"),
            // new SwerveDriveToShoot(),
            // new ConveyorShootRoutine(),

            // new FollowPathAndIntake("B to A"),
            new SwerveDriveToShoot(2.9)
                .withTolerance(0.05, 3),
            new ConveyorShootRoutine()
        );
    }
    
}
