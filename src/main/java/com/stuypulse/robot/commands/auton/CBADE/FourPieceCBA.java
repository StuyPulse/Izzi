package com.stuypulse.robot.commands.auton.CBADE;

import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.commands.vision.VisionChangeWhiteList;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceCBA extends SequentialCommandGroup {

    public FourPieceCBA() {
        addCommands(
            new VisionChangeWhiteList(7),
        
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),
                
                new FollowPathAlignAndShoot("Start To C")
            ),

            new FollowPathAndIntake("First Piece To C"),
            new WaitCommand(0.2),
            new FollowPathAlignAndShoot("C to CShoot"),

            new FollowPathAndIntake("CShoot To B"),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            new FollowPathAndIntake("B To A"),
            new SwerveDriveToShoot(),
            new ConveyorShootRoutine(),

            new VisionReloadWhiteList()
        );
    }
    
}
