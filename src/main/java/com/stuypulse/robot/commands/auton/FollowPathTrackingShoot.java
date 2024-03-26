package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.conveyor.ConveyorShoot;
import com.stuypulse.robot.commands.conveyor.ConveyorStop;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.streams.booleans.BStream;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FollowPathTrackingShoot extends ParallelCommandGroup {
    
    public FollowPathTrackingShoot(PathPlannerPath path, BStream shouldShoot) {
        addCommands(
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(path),
            new SequentialCommandGroup(
                new WaitUntilCommand(shouldShoot),
                new ConveyorShoot(),
                new WaitCommand(Settings.Conveyor.SHOOT_WAIT_DELAY.get()),
                new ConveyorStop(),
                new IntakeStop()
            )
        );
    }

}
