package com.stuypulse.robot.commands.auton.CBADE;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.DoNothingCommand;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ReroutableSixPieceCBAEF extends SequentialCommandGroup {

    public ReroutableSixPieceCBAEF(PathPlannerPath... paths) {

        PathPlannerPath E_TO_F = paths[7];
        
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(-15)
                    .withTolerance(0.03, 0.03, 3)
            ),

            // shoot preload
            new ConveyorShootRoutine(0.55),

            // intake C, shoot C
            new FollowPathAndIntake(paths[0]),
            new SwerveDriveToShoot()
                .withTolerance(0.03, 7),
            new ConveyorShootRoutine(),

            // intake B, shoot B
            new FollowPathAndIntake(paths[1]),
            new SwerveDriveToShoot()
                .withTolerance(0.05, 7),
            new ConveyorShootRoutine(),

            // intake A, shoot A
            new FollowPathAndIntake(paths[2]),
            new SwerveDriveToShoot()
                .withTolerance(0.05, 5),
            new ConveyorShootRoutine(0.6),

            // intake E
            new FollowPathAndIntake(paths[3]),
            
            new ConditionalCommand(
                // shoot E, intake F, shoot F
                new SequentialCommandGroup(
                    new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot()
                        .withTolerance(0.033, 7)),

                    new FollowPathAndIntake(paths[5]),

                    new ConditionalCommand(
                        new FollowPathAlignAndShoot(paths[6], new SwerveDriveToShoot()
                            .withTolerance(0.033, 7)),
                        new DoNothingCommand(),
                        Intake.getInstance()::hasNote)),

                // intake F, shoot F
                new SequentialCommandGroup(
                    new FollowPathAndIntake(E_TO_F),

                    new ConditionalCommand(
                        new FollowPathAlignAndShoot(paths[6], new SwerveDriveToShoot()
                            .withTolerance(0.033, 7)),
                        new DoNothingCommand(),
                        Intake.getInstance()::hasNote)
                ),
                Intake.getInstance()::hasNote)
        );
    }


}
