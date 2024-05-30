package com.stuypulse.robot.commands.auton.CBADE;

import com.pathplanner.lib.path.PathPlannerPath;


public class FivePieceChoreoCBAE extends SequentialCommandGroup {

    public FivePieceChoreoCBAE(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(-15)
                    .withTolerance(0.06, 0.06, 3)
            ),

            new ConveyorShootRoutine(0.55),

            new FollowPathAndIntake(paths[0]),
            new SwerveDriveToShoot()
                .withTolerance(0.03, 7),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[0]),
            new SwerveDriveToShoot()
                .withTolerance(0.03, 7),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[1]),
            new SwerveDriveToShoot()
                .withTolerance(0.05, 7),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[2]),
            new SwerveDriveToShoot()
                .withTolerance(0.05, 5),
            new ConveyorShootRoutine(0.6),

            new FollowPathAndIntake(paths[3]),
            new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot()
                .withTolerance(0.06, 7)),

        );

            
    }
}