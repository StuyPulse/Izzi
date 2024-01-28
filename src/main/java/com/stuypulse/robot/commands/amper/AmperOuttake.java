package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;

public class AmperOuttake extends Command {

    private final Amper amper;

    public AmperOuttake() {
        amper = Amper.getInstance();
        addRequirements(amper);
    } 

    @Override
    public void initialize() {
        amper.score();
    }

    @Override
    public boolean isFinished() {
        return !amper.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        amper.stopRoller();
    }
}
