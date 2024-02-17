/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AmperStop extends InstantCommand {

    private final Amper amper;

    public AmperStop() {
        amper = Amper.getInstance();
        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.stopRoller();
    }
}
