/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;

public class AmperToConveyor extends Command {

    private final Amper amper;

    public AmperToConveyor() {
        amper = Amper.getInstance();
        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.toConveyor();
    }

    // @Override
    // public boolean isFinished() {
    //     return amper.hasNote();
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     amper.stopRoller();
    // }
}
