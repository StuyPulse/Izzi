package com.stuypulse.robot.commands.auton.ADE;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.DoNothingCommand;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreePieceADE extends SequentialCommandGroup {

    public ThreePieceADE(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(18)
            ),

            // shoot preload
            new ConveyorShootRoutine(),

            // intake A
            new FollowPathAndIntake(paths[0]),
            // shoot A
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            // intake D
            new FollowPathAndIntake(paths[1]),
            // ferry D
            new PrintCommand("Ferry D"),
            new FollowPathAlignAndShoot(paths[2], new DoNothingCommand()),

            new PrintCommand("Intake E"),
            // intake E
            new FollowPathAndIntake(paths[3]),
            // shoot E
            new FollowPathAlignAndShoot(paths[4], SwerveDriveToShoot.withHigherDebounce())
        );
    }

}
