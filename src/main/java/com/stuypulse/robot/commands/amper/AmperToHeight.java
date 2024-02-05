package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AmperToHeight extends InstantCommand {

    public static Command untilDone(double height) {
        return new AmperToHeight(height)
            .until(() -> Amper.getInstance().isAtTargetHeight(Lift.MAX_HEIGHT_ERROR));
    }

    private final Amper amper;
    private final double height;

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
