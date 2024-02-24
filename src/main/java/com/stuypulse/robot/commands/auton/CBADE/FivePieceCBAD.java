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

public class FivePieceCBAD extends SequentialCommandGroup {

    public FivePieceCBAD() {
        addCommands(
            new ParallelCommandGroup(
                // new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                //     .andThen(new ShooterPodiumShot()),
                
                new FollowPathAlignAndShoot("Start To C", -40)
            ),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("First Piece To C"),
            new FollowPathAlignAndShoot("C to CShoot", -5),

            new FollowPathAndIntake("CShoot To B"),
            new SwerveDriveToShoot(5),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("B To A"),
            new SwerveDriveToShoot(40),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("A To D"),
            new FollowPathAlignAndShoot("D To Shoot", 30)
        );
    }
    
}
