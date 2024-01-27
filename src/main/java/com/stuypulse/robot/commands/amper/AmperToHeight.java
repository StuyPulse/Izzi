package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AmperToHeight extends InstantCommand {
    private final Amper amper;
    private double height;

    public AmperToHeight(double height) {
        amper = Amper.getInstance();
        this.height = height;
        
        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.setTargetHeight(height);
    }
}