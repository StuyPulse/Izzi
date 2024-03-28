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

public class BottomFerry extends SequentialCommandGroup {
    
    public BottomFerry(PathPlannerPath... paths) {
        addCommands(

            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                SwerveDriveToPose.speakerRelative(-45)
            ),

            new WaitCommand(Auton.SHOOTER_STARTUP_DELAY),

            new FollowPathAndIntake(paths[0]),
            SwerveDrive.getInstance().followPathCommand(paths[1]),

            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot(),

            new FollowPathAndIntake(paths[2]),
            SwerveDrive.getInstance().followPathCommand(paths[3]),

            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot(), 


            new FollowPathAndIntake(paths[4]),
            SwerveDrive.getInstance().followPathCommand(paths[5]),

            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot(), 

            new FollowPathAndIntake(paths[6]),
            SwerveDrive.getInstance().followPathCommand(paths[7]),

            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot(), 

            new FollowPathAndIntake(paths[8]),
            SwerveDrive.getInstance().followPathCommand(paths[9]),
            
            new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
            new ShooterPodiumShot()
        ); 
    }
}
