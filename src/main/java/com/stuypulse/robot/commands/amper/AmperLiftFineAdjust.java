package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class AmperLiftFineAdjust extends Command {
    private final Amper amper;
    private final Gamepad gamepad;

    public AmperLiftFineAdjust(Gamepad gamepad) {
        amper = Amper.getInstance();
        this.gamepad = gamepad;

        addRequirements(amper);
    }

    @Override
    public void execute() {
        if (gamepad.getRawDPadUp()) {
            amper.setTargetHeight(amper.getTargetHeight() + Units.inchesToMeters(1));
        } 
        else if (gamepad.getRawDPadDown()) {
            amper.setTargetHeight(amper.getLiftHeight() - Units.inchesToMeters(1));
        } 
    }
}
