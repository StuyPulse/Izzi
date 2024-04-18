package com.stuypulse.robot.commands.auton.HGF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.DoNothingCommand;
import com.stuypulse.robot.commands.FastAlignShootSpeakerRelative;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShootFast;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ReroutableFourPieceHGF extends SequentialCommandGroup {

    public ReroutableFourPieceHGF(PathPlannerPath... paths) {
        PathPlannerPath H_TO_G = paths[6];
        PathPlannerPath G_TO_F = paths[7];

        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(0.25)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(-55)
                    .withTolerance(0.1, 0.1, 2)
            ),

            new ShooterWaitForTarget(),
            ConveyorShootRoutine.untilNoteShot(0.75),

            // intake H
            new FollowPathAndIntake(paths[0]),
            new ConditionalCommand(
                // has H
                new SequentialCommandGroup(
                    // shoot H, intake G
                    // new FollowPathAlignAndShootFast(paths[1], new FastAlignShootSpeakerRelative(-45, 1.0)),
                    new FollowPathAlignAndShoot(paths[1], SwerveDriveToPose.speakerRelative(-45)),
                    new FollowPathAndIntake(paths[2]),
                    new ConditionalCommand(
                        // shoot G, intake F, shoot F
                        new SequentialCommandGroup(
                            // new FollowPathAlignAndShootFast(paths[3], new FastAlignShootSpeakerRelative(-45)),
                            new FollowPathAlignAndShoot(paths[3], SwerveDriveToPose.speakerRelative(-45)
                                .withTolerance(0.03, 0.03, 3)),
                            new FollowPathAndIntake(paths[4]),
                            
                            new ConditionalCommand(
                                new FollowPathAlignAndShoot(paths[5], SwerveDriveToPose.speakerRelative(-45)
                                    .withTolerance(0.03, 0.03, 3)),
                                new DoNothingCommand(),
                                Intake.getInstance()::hasNote)),

                        // intake F, shoot F
                        new SequentialCommandGroup(
                            new FollowPathAndIntake(G_TO_F),
                            
                            new ConditionalCommand(
                                new FollowPathAlignAndShoot(paths[5], SwerveDriveToPose.speakerRelative(-45)
                                    .withTolerance(0.03, 0.03, 3)),
                                new DoNothingCommand(),
                                Intake.getInstance()::hasNote)),
                        Intake.getInstance()::hasNote)),
                // no H
                new SequentialCommandGroup(
                    // intake G
                    new FollowPathAndIntake(H_TO_G),
                    new ConditionalCommand(
                        // shoot G, intake F, shoot F
                        new SequentialCommandGroup(
                            // new FollowPathAlignAndShootFast(paths[3], new FastAlignShootSpeakerRelative(-45)),
                            new FollowPathAlignAndShoot(paths[3], SwerveDriveToPose.speakerRelative(-45)
                                .withTolerance(0.03, 0.03, 3)),
                            new FollowPathAndIntake(paths[4]),

                            new ConditionalCommand(
                                new FollowPathAlignAndShoot(paths[5], SwerveDriveToPose.speakerRelative(-45)
                                    .withTolerance(0.03, 0.03, 3)),
                                new DoNothingCommand(),
                                Intake.getInstance()::hasNote)
                        ),

                        // intake F, shoot F
                        new SequentialCommandGroup(
                            new FollowPathAndIntake(G_TO_F),
                            
                            new ConditionalCommand(
                                new FollowPathAlignAndShoot(paths[5], SwerveDriveToPose.speakerRelative(-45)
                                    .withTolerance(0.03, 0.03, 3)),
                                new DoNothingCommand(),
                                Intake.getInstance()::hasNote)),

                        Intake.getInstance()::hasNote)),

                Intake.getInstance()::hasNote)
        );
    }

}
