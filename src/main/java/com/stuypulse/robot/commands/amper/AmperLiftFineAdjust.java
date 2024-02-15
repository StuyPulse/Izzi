/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.stuylib.input.Gamepad;

import com.stuypulse.robot.constants.Settings.Operator;
import com.stuypulse.robot.subsystems.amper.Amper;

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
            amper.setTargetHeight(amper.getLiftHeight() + Operator.LIFT_ADJUST_SPEED.get());
        } else if (gamepad.getRawDPadDown()) {
            amper.setTargetHeight(amper.getLiftHeight() - Operator.LIFT_ADJUST_SPEED.get());
        }
    }
}
