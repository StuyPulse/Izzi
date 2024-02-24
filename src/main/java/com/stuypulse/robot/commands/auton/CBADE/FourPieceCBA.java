package com.stuypulse.robot.commands.auton.CBADE;

import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourPieceCBA extends SequentialCommandGroup {

    public FourPieceCBA() {
        addCommands(
            new FollowPathAlignAndShoot("Start To C", -45),

            new FollowPathAndIntake("First Piece To C"),
            new SwerveDriveToShoot(-5),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("C To B"),
            new SwerveDriveToShoot(5),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("B To A"),
            new SwerveDriveToShoot(40),
            new ConveyorShootRoutine()
        );
    }
    
}
