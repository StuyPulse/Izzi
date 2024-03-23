package com.stuypulse.robot.commands.auton.CBADE;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SixPieceCBAED extends SequentialCommandGroup {
    
    
    public SixPieceCBAED(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(-45)
            ),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[0]),
            new ShooterPodiumShot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[1]),
            new ShooterPodiumShot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[2]),
            new ShooterPodiumShot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[3]),

            SwerveDrive.getInstance().followPathCommand(paths[4]),
            new ShooterPodiumShot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[5]),

            SwerveDrive.getInstance().followPathCommand(paths[6]),
            new ShooterPodiumShot(),
            new ConveyorShootRoutine()

        );
    }

}
