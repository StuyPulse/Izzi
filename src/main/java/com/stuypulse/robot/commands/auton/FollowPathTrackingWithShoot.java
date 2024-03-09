package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.CBADE.FivePieceCBAD;
import com.stuypulse.robot.commands.conveyor.ConveyorStop;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathTrackingWithShoot extends ParallelCommandGroup {

    public double getPathTime(PathPlannerPath[] paths) {
        return paths.getTrajectory(new ChassisSpeeds(), paths.getStartingDifferentialPose().getRotation())
            .getTotalTimeSeconds();
    }

    public FollowPathTrackingWithShoot(PathPlannerPath[] paths) {
        addCommands(
            new ParallelCommandGroup(
                SwerveDrive.getInstance().followPathCommand(paths),
                new WaitCommand(getPathTime(paths) - Auton.SHOOTER_START_PRE)
                    .andThen(new ShooterPodiumShot())
            ),
            new IntakeStop(),
            new ConveyorStop(),
            new ShooterStop()
            
        );
    }
}