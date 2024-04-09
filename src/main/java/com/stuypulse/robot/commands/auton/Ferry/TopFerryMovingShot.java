package com.stuypulse.robot.commands.auton.Ferry;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.auton.FollowPathFerryIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TopFerryMovingShot extends SequentialCommandGroup {
    public TopFerryMovingShot(PathPlannerPath... paths) {
        addCommands (
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(45)
            ),

            new ConveyorShootRoutine(),

            new ShooterSetRPM(Settings.Shooter.WING_FERRY),

            // intake D
            new FollowPathAndIntake(paths[0]),

            // shoot D, intake E
            new FollowPathFerryIntake(paths[1], 0.25),

            // shoot E, intake F
            new FollowPathFerryIntake(paths[2], 0.25),

            new ShooterPodiumShot(),

            // shoot F
            new FollowPathAlignAndShoot(paths[3], new SwerveDriveToShoot())
        );
    }
}
