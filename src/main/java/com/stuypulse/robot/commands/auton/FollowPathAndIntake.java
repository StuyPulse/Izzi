package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathAndIntake extends ParallelRaceGroup {

    public FollowPathAndIntake(String path) {
        this(path, Auton.DEFAULT_INTAKE_TIMEOUT);
    }
    
    public FollowPathAndIntake(String path, double intakeTimeout) {
        addCommands(
            new IntakeAcquire(),

            SwerveDrive.getInstance().followPathCommand(path)
                .andThen(new WaitCommand(intakeTimeout))
        );
    }

}
