package com.stuypulse.robot.util;

import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathReroute {
    private final FollowPathAndIntake intakeCommand;
    private final Command nextPathCommand;

    private SequentialCommandGroup command;

    public PathReroute(FollowPathAndIntake intakeCommand, SequentialCommandGroup command, Command nextPathCommand) {
        this.intakeCommand = intakeCommand;
        this.command = command;
        this.nextPathCommand = nextPathCommand;

        //Logging to SmartDashboard for testing
        //SmartDashboard.putString("Intake/Current Path Command", intakeCommand.getName());
        //SmartDashboard.putString("Intake/Next Path Command", nextPathCommand.getName());

    }

    public SequentialCommandGroup reroute() {
        if (intakeCommand.isFinished() && !Intake.getInstance().hasNote()) {
            command = new SequentialCommandGroup(nextPathCommand);
            return command;
        } 
        else {
            command.addCommands(nextPathCommand);
            return command;
        }
    }
}
