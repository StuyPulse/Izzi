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

    /**
     * Constructs a new ChoosePath object.
     * @param noteCommand The command to execute if the intake has a note at the end of the path.
     * @param noNoteCommand The command to execute if the intake does not have a note at the end of the path.
     */
    public ChoosePath(Command noteCommand, Command noNoteCommand) {
        super(noteCommand, noNoteCommand, Intake.getInstance()::hasNote);
    }

    public ChoosePath(PathPlannerPath scorePath, PathPlannerPath intakePath) {
        this(
            new SequentialCommandGroup(
                new FollowPathAndIntake(scorePath), 
                new ConveyorShootRoutine(),
                new ShooterPodiumShot()
            ), 
            new SequentialCommandGroup(
                new ConveyorShootRoutine(),
                new ShooterPodiumShot()
            )
        );
    }
}
