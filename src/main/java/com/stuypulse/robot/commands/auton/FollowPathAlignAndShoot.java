package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FollowPathAlignAndShoot extends SequentialCommandGroup {
    
    public FollowPathAlignAndShoot(String path) {
        addCommands(
            SwerveDrive.getInstance().followPathCommand("H To HShoot (HGF)"),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine()
        );
    }

}
