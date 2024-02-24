package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathAlignAndShoot extends SequentialCommandGroup {

    public double getPathTime(String path) {
        PathPlannerPath p = PathPlannerPath.fromPathFile(path);
        return p.getTrajectory(new ChassisSpeeds(), p.getStartingDifferentialPose().getRotation())
            .getTotalTimeSeconds();
    }

    public FollowPathAlignAndShoot(String path) {
        addCommands(
            new ParallelCommandGroup(
                SwerveDrive.getInstance().followPathCommand(path),
                new WaitCommand(getPathTime(path) - 1.0).andThen(new ShooterPodiumShot())
            ),
            new SwerveDriveToShoot(),
            new ShooterWaitForTarget().andThen(new ConveyorShootRoutine()),
            new ShooterStop()
        );
    }
    
    public FollowPathAlignAndShoot(String path, double angle) {
        addCommands(
            new ParallelCommandGroup(
                SwerveDrive.getInstance().followPathCommand(path),
                new WaitCommand(getPathTime(path) - 1.0).andThen(new ShooterPodiumShot())
            ),
            new SwerveDriveToShoot(angle),
            new ShooterWaitForTarget().andThen(new ConveyorShootRoutine()),
            new ShooterStop()
        );
    }

}
