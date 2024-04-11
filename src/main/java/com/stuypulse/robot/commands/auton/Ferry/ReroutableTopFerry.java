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

public class ReroutableTopFerry extends SequentialCommandGroup {

    public ReroutableTopFerry(PathPlannerPath... paths) {

        PathPlannerPath D_TO_E = paths[6];
        PathPlannerPath E_TO_F = paths[7];

        addCommands (
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(45)
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
                    new FollowPathAndIntake(paths[2]),

                    new ConditionalCommand(
                        // if we have a note, we can shoot E
                        new SequentialCommandGroup(

                            // shoot E, intake F
                            SwerveDrive.getInstance().followPathCommand(paths[3]),
                            new SwerveDriveStop(),
                            new ConveyorShootRoutine(),
                            new ShooterPodiumShot(),

                            new FollowPathAndIntake(paths[4]),

                            new ConditionalCommand(
                                // if we have a note, we can shoot F
                                new FollowPathAlignAndShoot(paths[5], new SwerveDriveToShoot()), 
                                new InstantCommand(), 
                                Intake.getInstance()::hasNote)
                        ), 
                        // else intake E
                        new SequentialCommandGroup(
                            new FollowPathAndIntake(D_TO_E),

                            new ConditionalCommand(
                                new SequentialCommandGroup(
                                    // shoot E, intake F
                                    SwerveDrive.getInstance().followPathCommand(paths[3]),
                                    new SwerveDriveStop(),
                                    new ConveyorShootRoutine(),
                                    new ShooterPodiumShot(),
                                    
                                    new FollowPathAndIntake(paths[4]),
                                    
                                    new ConditionalCommand(
                                        // if we have a note, we can shoot F
                                        new FollowPathAlignAndShoot(paths[5], new SwerveDriveToShoot()), 
                                        new InstantCommand(), 
                                        Intake.getInstance()::hasNote)
                                ), 
                                new SequentialCommandGroup(
                                    new FollowPathAndIntake(E_TO_F),
                                    
                                    new ConditionalCommand(
                                        // if we have a note, we can shoot F
                                        new FollowPathAlignAndShoot(paths[5], new SwerveDriveToShoot()), 
                                        new InstantCommand(), 
                                        Intake.getInstance()::hasNote)
                                ),
                                Intake.getInstance()::hasNote)
                        ), 
                        Intake.getInstance()::hasNote)
                ), 
                // else intake E
                new SequentialCommandGroup(
                    new FollowPathAndIntake(D_TO_E),

                    new ConditionalCommand(
                        new SequentialCommandGroup(
                            // shoot E, intake F
                            SwerveDrive.getInstance().followPathCommand(paths[3]),
                            new SwerveDriveStop(),
                            new ConveyorShootRoutine(),
                            new ShooterPodiumShot(),
                            
                            new FollowPathAndIntake(paths[4]),
                            
                            new ConditionalCommand(
                                // if we have a note, we can shoot F
                                new FollowPathAlignAndShoot(paths[5], new SwerveDriveToShoot()), 
                                new InstantCommand(), 
                                Intake.getInstance()::hasNote)
                        ), 
                        new SequentialCommandGroup(
                            new FollowPathAndIntake(E_TO_F),
                            
                            new ConditionalCommand(
                                // if we have a note, we can shoot F
                                new FollowPathAlignAndShoot(paths[5], new SwerveDriveToShoot()), 
                                new InstantCommand(), 
                                Intake.getInstance()::hasNote)
                        ), 
                        Intake.getInstance()::hasNote)
                ), 
                Intake.getInstance()::hasNote)
        );
    }
}
