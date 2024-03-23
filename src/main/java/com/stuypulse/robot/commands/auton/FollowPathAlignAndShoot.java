package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.conveyor.ConveyorShoot;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.conveyor.ConveyorStop;
import com.stuypulse.robot.commands.conveyor.ConveyorToShooter;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathAlignAndShoot extends SequentialCommandGroup {

    public double getPathTime(PathPlannerPath path) {
        return path.getTrajectory(new ChassisSpeeds(), path.getStartingDifferentialPose().getRotation())
            .getTotalTimeSeconds();
    }

    public FollowPathAlignAndShoot(PathPlannerPath path, Command alignCommand) {
        addCommands(
            new ParallelCommandGroup(
                new ConveyorToShooter()
                    .withTimeout(3.0),
                SwerveDrive.getInstance().followPathCommand(path),
                new WaitCommand(getPathTime(path) - Auton.SHOOTER_START_PRE)
                    .andThen(new ShooterPodiumShot())
            ),
            alignCommand,
            new ShooterWaitForTarget(),
            new ConveyorShoot(),
            new WaitCommand(Settings.Conveyor.SHOOT_WAIT_DELAY.get()),
            // ConveyorShoot.untilDone()
            //     .withTimeout(Settings.Conveyor.SHOOT_WAIT_DELAY.get()),
            new ConveyorStop(),
            new IntakeStop()
        );
    }

}
