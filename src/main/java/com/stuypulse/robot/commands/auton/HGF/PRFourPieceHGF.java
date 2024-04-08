package com.stuypulse.robot.commands.auton.HGF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.util.ChoosePath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PRFourPieceHGF extends SequentialCommandGroup {

    public PRFourPieceHGF(PathPlannerPath... paths) {
        addCommands(
            new ShooterPodiumShot(),

            new ShooterWaitForTarget()
                .withTimeout(1.0),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[0]),
            new FollowPathAlignAndShoot(paths[1], SwerveDriveToShoot.withHigherDebounce()),

            new FollowPathAndIntake(paths[2]),

            new ChoosePath(
                new SequentialCommandGroup(
                    new FollowPathAlignAndShoot(paths[3], SwerveDriveToShoot.withHigherDebounce()), 
                    new FollowPathAndIntake(paths[4]),
                    new FollowPathAlignAndShoot(paths[5], SwerveDriveToShoot.withHigherDebounce())
                ),
                new SequentialCommandGroup(
                    new FollowPathAndIntake(PathPlannerPath.fromPathFile("G to F")),
                    new FollowPathAlignAndShoot(paths[5], SwerveDriveToShoot.withHigherDebounce())
                ) 
            )
        );
    }

}
