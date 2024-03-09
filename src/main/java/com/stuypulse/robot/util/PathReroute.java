package com.stuypulse.robot.util;

import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathReroute extends Command {
    private final FollowPathAndIntake intakeCommand;
    private final Command nextPathCommand;

    private SequentialCommandGroup auto;
    private SequentialCommandGroup command;

    private static boolean hasIntaked;

    public PathReroute(SequentialCommandGroup auto, FollowPathAndIntake intakeCommand, SequentialCommandGroup command, Command nextPathCommand) {
        this.auto = auto;
        this.intakeCommand = intakeCommand;
        this.command = command;
        this.nextPathCommand = nextPathCommand;
        hasIntaked = false;

        //Logging to SmartDashboard for testin\g
        //SmartDashboard.putString("Intake/Current Path Command", intakeCommand.getName());
        //SmartDashboard.putString("Intake/Next Path Command", nextPathCommand.getName());

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        hasIntaked = (intakeCommand.isFinished() && !Intake.getInstance().hasNote());
    }

    public SequentialCommandGroup reroute() {
        System.out.println("RAN REROUTE METHOD");
        if (intakeCommand.isFinished() && !Intake.getInstance().hasNote()) {
            command = new SequentialCommandGroup(nextPathCommand);
            return command;
        } 
        else {
            command.addCommands(nextPathCommand);
            return command;
        }
    }

    /*
     *             new ConditionalCommand(
                new SequentialCommandGroup(
                    new SwerveDriveToShoot(2.9),
                    new ConveyorShootRoutine()
                ),
                new DoNothing, 
                () -> (intakeCommand.isFinished() && !Intake.getInstance().hasNote()));
     */

    @Override
    public void end(boolean isFinished) {
        
    }
}
