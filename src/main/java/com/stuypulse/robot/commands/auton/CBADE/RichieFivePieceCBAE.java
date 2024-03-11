package com.stuypulse.robot.commands.auton.CBADE;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.util.ChoosePath;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RichieFivePieceCBAE extends SequentialCommandGroup {

    public RichieFivePieceCBAE(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(-18)
            ),

            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[0]),
            new SwerveDriveToShoot(2.9),
            new ConveyorShootRoutine(),
        
            new FollowPathAndIntake(paths[1]),
            new SwerveDriveToShoot(2.9),
            new ConveyorShootRoutine(),


            new FollowPathAndIntake(paths[2]),
            new SwerveDriveToShoot(2.9)
                .withTolerance(0.05, 3),
            new ConveyorShootRoutine(),

           
            //new FollowPathAndIntake(paths[3]),
            //new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot())

            new FollowPathAndIntake(paths[3]),
            new ChoosePath(
                new SequentialCommandGroup(
                    new FollowPathAndIntake(PathPlannerPath.fromPathFile("E to F")),
                    new FollowPathAlignAndShoot(PathPlannerPath.fromPathFile("F To Shoot (HGF)"), new SwerveDriveToShoot())
                ),
                new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot())
            )

        );
        
    }
}
