package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;

public class RunRoller extends Command {

    public Amper amper;
    public boolean forward;

    public RunRoller(boolean forward) {
        amper = Amper.getInstance();
        this.forward = forward;

        addRequirements(amper);
    } 

    @Override
    public void initialize() {
        if (forward) {
            amper.score();
        }
        else {
            amper.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        amper.stopRoller();
    }
}
