package com.stuypulse.robot.commands.auton.DE;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceDE extends SequentialCommandGroup {

    public TwoPieceDE(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(18)
            ),

            // shoot preload
            new ConveyorShootRoutine(),

            // intake D
            new FollowPathAndIntake(paths[0]),
            // ferry D
            new FollowPathAlignAndShoot(paths[1], new DoNothingCommand()),

            // intake E
            new FollowPathAndIntake(paths[2]),
            // shoot E
            new FollowPathAlignAndShoot(paths[3], SwerveDriveToShoot.withHigherDebounce())
        );
    }

}
