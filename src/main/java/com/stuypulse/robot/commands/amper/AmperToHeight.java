/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AmperToHeight extends InstantCommand {

    public static Command untilDone(double height) {
        return new AmperToHeight(height)
            .andThen(new WaitUntilCommand(() -> Amper.getInstance().isAtTargetHeight(Lift.MAX_HEIGHT_ERROR)));
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
