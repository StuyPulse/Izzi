package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AmperScoreForever extends InstantCommand {

    private final Amper amper;

    public AmperScoreForever() {
        amper = Amper.getInstance();
        addRequirements(amper);
    } 

    @Override
    public void initialize() {
        amper.score();
    }
}
