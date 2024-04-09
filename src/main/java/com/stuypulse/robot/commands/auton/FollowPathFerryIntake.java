package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.conveyor.ConveyorShoot;
import com.stuypulse.robot.commands.conveyor.ConveyorStop;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathFerryIntake extends SequentialCommandGroup {
    
    public double getPathTime(PathPlannerPath path) {
        return path.getTrajectory(new ChassisSpeeds(), path.getStartingDifferentialPose().getRotation())
            .getTotalTimeSeconds();
    }

    public FollowPathFerryIntake(PathPlannerPath path, double shootTimePercent) {
        shootTimePercent = MathUtil.clamp(shootTimePercent, 0, 1);

        double pathTime = getPathTime(path);
        double shootTime = pathTime * shootTimePercent;

        // make deadline group on intake if we need time
        addCommands(
            SwerveDrive.getInstance().followPathCommand(path),
            
            new SequentialCommandGroup(
                new WaitCommand(shootTime),
                new ConveyorShoot(),
                // longer b/c current draw from driving?
                new WaitCommand(0.6),
                new ConveyorStop(),
                new IntakeAcquire()
            )
        );
    }

}
