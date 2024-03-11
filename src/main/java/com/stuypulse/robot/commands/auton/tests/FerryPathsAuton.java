package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FerryPathsAuton extends SequentialCommandGroup {
    
    public FerryPathsAuton(PathPlannerPath... paths) {
        addCommands(

            new WaitCommand(Auton.SHOOTER_STARTUP_DELAY),

            new FollowPathAndIntake(paths[0]),
            new ShooterPodiumShot(),
            new FollowPathAndIntake(paths[1]),
            new ShooterPodiumShot(),
            new FollowPathAndIntake(paths[2]),
            new ShooterPodiumShot(),
            new FollowPathAndIntake(paths[3]),
            new ShooterPodiumShot(),
            new FollowPathAndIntake(paths[4]),
            new ShooterPodiumShot(),
            new FollowPathAndIntake(paths[5]),
            new ShooterPodiumShot(),
            new FollowPathAndIntake(paths[6]),
            new ShooterPodiumShot()

        );
    }
}
