package com.stuypulse.robot.commands.auton.HGF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.FastAlignShootSpeakerRelative;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShootFast;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceHGF extends SequentialCommandGroup {

    public FourPieceHGF(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(0.25)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(-55)
                    .withTolerance(0.1, 0.1, 2)
            ),

            new ShooterWaitForTarget(),
            ConveyorShootRoutine.untilNoteShot(0.75),

            new FollowPathAndIntake(paths[0]),
            // new FollowPathAlignAndShootFast(paths[1], new FastAlignShootSpeakerRelative(-45, 1.0)),
            new FollowPathAlignAndShoot(paths[1], SwerveDriveToPose.speakerRelative(-45)),
            new FollowPathAndIntake(paths[2]),
            // new FollowPathAlignAndShootFast(paths[3], new FastAlignShootSpeakerRelative(-45)),
            new FollowPathAlignAndShoot(paths[3], SwerveDriveToPose.speakerRelative(-45)),
            new FollowPathAndIntake(paths[4]),
            new FollowPathAlignAndShoot(paths[5], SwerveDriveToPose.speakerRelative(-45))
        );
    }

}
