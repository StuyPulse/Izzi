package com.stuypulse.robot.util;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.conveyor.ConveyorShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/**
 * This class represents a command that chooses a path based on the intake IR.
 * It extends the ConditionalCommand class.
 */

public class ChoosePath extends ConditionalCommand {

    private Optional<Command> currentPathCommand;
    /**
     * Constructs a new ChoosePath object.
     * 
     * @param nextPathCommand The command to execute if the intake does not have a note at the end of the path.
     * @param sequentialCommand The command to execute if the intake has a note at the end of the path.
     */

    public ChoosePath(SequentialCommandGroup nextPathCommand, Command sequentialCommand) {
        super(nextPathCommand, sequentialCommand, () -> !Intake.getInstance().hasNote());
        this.currentPathCommand = Optional.empty();
    }

    public ChoosePath(PathPlannerPath nextPath) {
        super(
            new SequentialCommandGroup(
                new FollowPathAndIntake(nextPath), 
                new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
                new ShooterPodiumShot()
            ), 
            new SequentialCommandGroup(
                new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
                new ShooterPodiumShot()
            ), 
            () -> !Intake.getInstance().hasNote());
        this.currentPathCommand = Optional.empty(); 
    }

    
    public ChoosePath(Command currentPathCommand, PathPlannerPath nextPath) {
        super(
            new SequentialCommandGroup(
                new FollowPathAndIntake(nextPath), 
                new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
                new ShooterPodiumShot()
            ), 
            new SequentialCommandGroup(
                new ConveyorShootRoutine(Settings.Conveyor.SHOOT_WAIT_DELAY.getAsDouble()),
                new ShooterPodiumShot()
            ), 
            () -> !Intake.getInstance().hasNote());
        this.currentPathCommand = Optional.of(currentPathCommand); 
    }

    public Command getCommand() {
        if (currentPathCommand.isPresent()) {
            return new SequentialCommandGroup(
                currentPathCommand.get(),
                this
            );
        } else {
            return this;
        }
    }
}
