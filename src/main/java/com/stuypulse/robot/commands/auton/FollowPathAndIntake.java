package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathAndIntake extends SequentialCommandGroup {

    public FollowPathAndIntake(PathPlannerPath path) {
        this(path, Auton.DEFAULT_INTAKE_TIMEOUT);
    }
    
    public FollowPathAndIntake(PathPlannerPath path, double intakeTimeout) {
        addCommands(
            new ParallelRaceGroup(
                new IntakeAcquire(),
                SwerveDrive.getInstance().followPathCommand(path)
                    .andThen(new WaitCommand(intakeTimeout))
            ),

            new InstantCommand(() -> SwerveDrive.getInstance().stop())
        );
    }

}
