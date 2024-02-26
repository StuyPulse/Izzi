package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathAlignAndShoot extends SequentialCommandGroup {

    public double getPathTime(String path) {
        PathPlannerPath p = PathPlannerPath.fromPathFile(path);
        return p.getTrajectory(new ChassisSpeeds(), p.getStartingDifferentialPose().getRotation())
            .getTotalTimeSeconds();
    }

    public FollowPathAlignAndShoot(String path, Command alignCommand) {
        addCommands(
            new ParallelCommandGroup(
                SwerveDrive.getInstance().followPathCommand(path),
                new WaitCommand(getPathTime(path) - Auton.SHOOTER_START_PRE)
                    .andThen(new ShooterPodiumShot())
            ),
            alignCommand,
            new ShooterWaitForTarget().andThen(new ConveyorShootRoutine()),
            new ShooterStop()
        );
    }

}
