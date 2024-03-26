package com.stuypulse.robot.commands.auton.CHFG;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FivePieceCHGF extends SequentialCommandGroup {

     public FivePieceCHGF(PathPlannerPath... paths) {
        addCommands(
         
            new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                .andThen(new ShooterPodiumShot()),
        
            //shoot preload
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[0]),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake(paths[1]),
            new FollowPathAlignAndShoot(paths[2], new SwerveDriveToShoot()),

            new FollowPathAndIntake(paths[3]),
            new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot()),

            new FollowPathAndIntake(paths[5])
        );
    }

    
}
