package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathWithShoot extends ParallelCommandGroup {

    public FollowPathWithShoot(PathPlannerPath path, double shootTime) {
        addCommands(
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(path),
            new WaitCommand(shootTime)
                .andThen(new ShooterWaitForTarget())
                .andThen(new ConveyorShootRoutine())
        );
    }

}
