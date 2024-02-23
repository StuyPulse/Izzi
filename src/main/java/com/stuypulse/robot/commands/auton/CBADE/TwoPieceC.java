package com.stuypulse.robot.commands.auton.CBADE;

import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoPieceC extends SequentialCommandGroup {

    public TwoPieceC() {
        addCommands(
            new ParallelCommandGroup(
                new ShooterPodiumShot(),
                new SwerveDriveToShoot()
            ),
            new ConveyorShootRoutine(),

            SwerveDrive.getInstance().followPathCommand("First Piece To C").raceWith(new IntakeAcquire()),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine()
        );
    }
    
}
