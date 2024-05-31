
package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AmperScoreThenStop extends InstantCommand {

    private final Amper amper;
    
    public static Command scoreThenStop() {
        return new AmperScoreThenStop();
    }

    public AmperScoreThenStop() {
        amper = Amper.getInstance();
        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.amp();
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
