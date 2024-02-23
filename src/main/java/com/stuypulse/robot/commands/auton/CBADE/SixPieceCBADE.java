package com.stuypulse.robot.commands.auton.CBADE;

import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SixPieceCBADE extends SequentialCommandGroup {

    public SixPieceCBADE() {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                new SwerveDriveToShoot()
            ),
            new ConveyorShootRoutine(),

            new IntakeAcquire().withTimeout(Auton.DEFAULT_INTAKE_TIMEOUT),
            
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("C To B"),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("B To A"),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("A To D"),
            new FollowPathAlignAndShoot("D To Shoot"),

            new FollowPathAndIntake("Shoot To E"),
            new FollowPathAlignAndShoot("D To Shoot")
        );
    }
    
}
