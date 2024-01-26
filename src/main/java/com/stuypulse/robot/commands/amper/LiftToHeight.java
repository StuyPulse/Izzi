package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;


import edu.wpi.first.wpilibj2.command.Command;

public class LiftToHeight extends Command {
    private final Amper amper;
    private double height;
    private double threshold;

 
    public LiftToHeight(double height, double threshold) {
        amper = Amper.getInstance();
        this.height = height;
        this.threshold = threshold;
        
        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.setTargetHeight(height);
    }

    @Override
    public void end(boolean interrupted) {
        amper.stopLift();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(amper.getLiftHeight() - height) < threshold;
    }
}