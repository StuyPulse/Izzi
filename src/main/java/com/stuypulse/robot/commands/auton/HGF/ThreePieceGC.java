package com.stuypulse.robot.commands.auton.HGF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreePieceGC extends SequentialCommandGroup {

    public ThreePieceGC(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(0.25)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(-55)
                    .withTolerance(0.05, 0.05, 2)
            ),

            new ShooterWaitForTarget()
                .withTimeout(0.25),
            ConveyorShootRoutine.untilNoteShot(1.75),

            new FollowPathAndIntake(paths[0]),
            new FollowPathAlignAndShoot(paths[1], SwerveDriveToPose.speakerRelative(-45)
                .withTolerance(0.05, 0.05, 5)),
            
            new FollowPathAndIntake(paths[2]),
            new SwerveDriveToShoot()
                .withTolerance(0.03, 7),
            new ConveyorShootRoutine(0.7)
        );
    }

}
