package com.stuypulse.robot.util;
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
     * 
     * @param nextPathCommand The command to execute if the intake does not have a note at the end of the path.
     * @param sequentialCommand The command to execute if the intake has a note at the end of the path.
     */

    public ChoosePath(SequentialCommandGroup nextPathCommand, Command sequentialCommand) {
        super(nextPathCommand, sequentialCommand, () -> !Intake.getInstance().hasNote());
    }
}
