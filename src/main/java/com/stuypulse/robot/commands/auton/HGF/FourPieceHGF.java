package com.stuypulse.robot.commands.auton.HGF;

import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceHGF extends SequentialCommandGroup {

    public FourPieceHGF() {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),

                SwerveDriveToPose.speakerRelative(-50)
                    .withTolerance(0.1, 0.1, 2)
            ),

            new ShooterWaitForTarget().andThen(new ConveyorShootRoutine()),
            new ShooterStop(),

            new FollowPathAndIntake("Start To H (HGF)"),
            new FollowPathAlignAndShoot("H To HShoot (HGF)", SwerveDriveToPose.speakerRelative(-45)),
            new FollowPathAndIntake("HShoot To G (HGF)"),
            new FollowPathAlignAndShoot("G To Shoot (HGF)", SwerveDriveToPose.speakerRelative(-7)),
            new FollowPathAndIntake("GShoot To F (HGF)"),
            new FollowPathAlignAndShoot("F To Shoot (HGF)", SwerveDriveToPose.speakerRelative(-7)) // 1 second of shoot time?
        );
    }

}
