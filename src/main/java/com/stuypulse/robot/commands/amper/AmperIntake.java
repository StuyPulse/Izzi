package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;

public class AmperIntake extends Command {

    private final Amper amper;

    public AmperIntake() {
        amper = Amper.getInstance();
        addRequirements(amper);
    } 

    @Override
    public void initialize() {
        amper.intake();
    }

    @Override
    public void end(boolean interrupted) {
        amper.stopRoller();
    }
}
