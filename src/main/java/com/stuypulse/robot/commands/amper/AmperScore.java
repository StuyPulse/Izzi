package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AmperScore extends InstantCommand {

    public static Command forSeconds(double seconds) {
        return new AmperScore()
            .andThen(new WaitCommand(seconds))
            .andThen(new AmperStop());
    }

    private final Amper amper;

    public AmperScore() {
        amper = Amper.getInstance();
        addRequirements(amper);
    } 

    @Override
    public void initialize() {
        amper.score();
    }
}
