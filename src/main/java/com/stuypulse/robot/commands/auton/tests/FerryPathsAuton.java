package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FerryPathsAuton extends SequentialCommandGroup {
    
    public FerryPathsAuton(PathPlannerPath... paths) {
        addCommands(

            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(-45)
            ),

            new WaitCommand(Auton.SHOOTER_STARTUP_DELAY),

            new FollowPathAndIntake(paths[0]),
            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot(),

            SwerveDrive.getInstance().followPathCommand(paths[1]),
            
            new FollowPathAndIntake(paths[2]),
            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot(), 

            SwerveDrive.getInstance().followPathCommand(paths[3]),

            new FollowPathAndIntake(paths[4]),
            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot(), 

            SwerveDrive.getInstance().followPathCommand(paths[5]),

            new FollowPathAndIntake(paths[6]),
            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot(), 

            SwerveDrive.getInstance().followPathCommand(paths[7]),
            
            new FollowPathAndIntake(paths[8]),
            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot()
        ); 
    }
}
