package com.stuypulse.robot.commands.auton.Ferry;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.swerve.SwerveDriveStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// top ferry until E pickup, then pause for 3 seconds, go back and shoot
public class TopFerryM120 extends SequentialCommandGroup {
    public TopFerryM120(PathPlannerPath... paths) {
        addCommands (
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(42.5)
                    .withTolerance(0.03, 0.03, 3)
            ),

            new ConveyorShootRoutine(),

            new ShooterSetRPM(Settings.Shooter.WING_FERRY),

            // intake D
            new FollowPathAndIntake(paths[0]),

            new ConditionalCommand(
                // if we have a note, we can shoot D
                new SequentialCommandGroup(
                    // shoot D, intake E
                    SwerveDrive.getInstance().followPathCommand(paths[1]),
                    new SwerveDriveStop(),
                    new ConveyorShootRoutine(),
                    new FollowPathAndIntake(paths[2])
                ),
                // no note, reroute
                new FollowPathAndIntake(paths[4]),
                Intake.getInstance()::hasNote
            ),
            
            // wait 3 seconds
            new ShooterPodiumShot(),
            new WaitCommand(3.0),

            // shoot E
            new FollowPathAlignAndShoot(paths[3], new SwerveDriveToShoot()
                .withTolerance(0.06, 7))
        );
    }
}
