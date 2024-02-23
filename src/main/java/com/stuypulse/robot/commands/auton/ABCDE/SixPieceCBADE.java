package com.stuypulse.robot.commands.auton.ABCDE;

import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SixPieceCBADE extends SequentialCommandGroup {

    public SixPieceCBADE() {
        addCommands(
            new ParallelCommandGroup(
                new ShooterPodiumShot(),
                new SwerveDriveToShoot()
            ),
            new ConveyorShootRoutine(),

            SwerveDrive.getInstance().followPathCommand("First Piece To C").raceWith(new IntakeAcquire()),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            SwerveDrive.getInstance().followPathCommand("C To B").raceWith(new IntakeAcquire()),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            SwerveDrive.getInstance().followPathCommand("B To A").raceWith(new IntakeAcquire()),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            SwerveDrive.getInstance().followPathCommand("A To D").raceWith(new IntakeAcquire()),
            SwerveDrive.getInstance().followPathCommand("D To Shoot"),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            SwerveDrive.getInstance().followPathCommand("Shoot To E").raceWith(new IntakeAcquire()),
            SwerveDrive.getInstance().followPathCommand("D To Shoot"),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine()
        );
    }
    
}
