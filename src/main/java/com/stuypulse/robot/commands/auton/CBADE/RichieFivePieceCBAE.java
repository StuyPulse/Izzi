package com.stuypulse.robot.commands.auton.CBADE;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.DoNothingCommand;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.util.PathReroute;

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

            new PathReroute(
                this,
                new FollowPathAndIntake(paths[2]),
                new SequentialCommandGroup(
                    new SwerveDriveToShoot(2.9),
                    new ConveyorShootRoutine()
                ), new FollowPathAndIntake(paths[3])).reroute(),

            /*
                     new ChoosePathDirection(
                new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot()),
                new SequentialCommandGroup(
                    new FollowPathAndIntake
                    new FollowPathAndShoot
                )
            ), */    

            //XXX: This is a reroute, use DoNothingCommand if no follow up path is needed
            new PathReroute(
                this,
                new FollowPathAndIntake(paths[3]),
                new SequentialCommandGroup(
                    new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot())
                ), new DoNothingCommand()).reroute()
        );
        
    }
    
}
