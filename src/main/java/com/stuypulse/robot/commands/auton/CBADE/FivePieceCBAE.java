package com.stuypulse.robot.commands.auton.CBADE;

import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FivePieceCBAE extends SequentialCommandGroup {

    public FivePieceCBAE() {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
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

            SwerveDrive.getInstance().followPathCommand("A To E").raceWith(new IntakeAcquire()),
            SwerveDrive.getInstance().followPathCommand("E To Shoot"),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine()
        );
    }
    
}
