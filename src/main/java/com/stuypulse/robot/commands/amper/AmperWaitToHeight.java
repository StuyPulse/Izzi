package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;

public class AmperWaitToHeight extends Command {

    private final Amper amper;
    private final double height;

    public AmperWaitToHeight(double height) {
        amper = Amper.getInstance();
        this.height = height;

        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.setTargetHeight(height);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(amper.getLiftHeight() - height) < Lift.MAX_HEIGHT_ERROR;
    }
}
