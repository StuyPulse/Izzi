package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AmperScore extends InstantCommand {

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
