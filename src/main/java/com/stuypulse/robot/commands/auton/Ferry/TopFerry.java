package com.stuypulse.robot.commands.auton.Ferry;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TopFerry extends SequentialCommandGroup {
    public TopFerry(PathPlannerPath... paths) {
        addCommands (
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(45)
            ),

            new ConveyorShootRoutine(),

            // intake D
            new FollowPathAndIntake(paths[0]),

            // shoot D, intake E
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            new ConveyorShootRoutine(),
            new FollowPathAndIntake(paths[2]),

            // shoot E, intake F
            SwerveDrive.getInstance().followPathCommand(paths[3]),
            new ConveyorShootRoutine(),
            new FollowPathAndIntake(paths[4]),

            // shoot F
            new FollowPathAlignAndShoot(paths[5], new SwerveDriveToShoot())
        );
    }
}
