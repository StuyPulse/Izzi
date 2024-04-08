package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TopFerry extends SequentialCommandGroup {
    public TopFerry(PathPlannerPath... paths) {
        addCommands (
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(-45)
            ),

            new FollowPathAndIntake(paths[0]),
            SwerveDrive.getInstance().followPathWithAmpZoneAlignCommand(paths[1]),

            new FollowPathAndIntake(paths[2]),
            SwerveDrive.getInstance().followPathWithAmpZoneAlignCommand(paths[3]),

            new FollowPathAndIntake(paths[4]),
            new FollowPathAlignAndShoot(paths[5], new SwerveDriveToShoot()
                .withTolerance(0,0))
        );
    }
}
