package com.stuypulse.robot.commands.auton.HGF;

import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceGHF extends SequentialCommandGroup {

    public FourPieceGHF() {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterPodiumShot()),

                new SwerveDriveToShoot()
            ),

            new ConveyorShootRoutine(),

            new FollowPathAndIntake("Start To G (GHF)"),
            new FollowPathAlignAndShoot("G To GShoot (GHF)"),
            new FollowPathAndIntake("GShoot To H (GHF)"),
            new FollowPathAlignAndShoot("H To HShoot (HGF)"),
            new FollowPathAndIntake("HShoot To F (GHF)"),
            new FollowPathAlignAndShoot("F To Shoot (HGF)")
        );
    }

}
