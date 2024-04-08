package com.stuypulse.robot.commands.auton.CBADE;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootNoIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.conveyor.ConveyorStop;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SixPieceCBAEDOld extends SequentialCommandGroup {
    
    
    public SixPieceCBAEDOld(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),

                SwerveDrive.getInstance().followPathCommand(paths[0])
            ),

            new SwerveDriveToShoot()
                .withTolerance(100, 5),
            new ConveyorShootNoIntake(),
            new WaitCommand(0.55),
            new ConveyorStop(),

            new FollowPathAndIntake(paths[1]),
            new SwerveDriveToShoot()
                .withTolerance(0.03, 7),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[2]),
            new SwerveDriveToShoot()
                .withTolerance(0.05, 7),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[3]),
            new SwerveDriveToShoot()
                .withTolerance(0.05, 5),
            new ConveyorShootRoutine(0.6),

            new FollowPathAndIntake(paths[4]),
            new FollowPathAlignAndShoot(paths[5], new SwerveDriveToShoot()
                .withTolerance(0.033, 7)),

            new FollowPathAndIntake(paths[6]),
            new FollowPathAlignAndShoot(paths[7], new SwerveDriveToShoot()
                .withTolerance(0.033, 7))
        );
    }

}
