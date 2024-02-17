/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AmperSetLiftConstraints extends InstantCommand {

    private final Amper amper;

    private final double maxVelocity;
    private final double maxAcceleration;

    public AmperSetLiftConstraints() {
        this(Settings.Amper.Lift.VEL_LIMIT, Settings.Amper.Lift.ACCEL_LIMIT);
    }

    public AmperSetLiftConstraints(double maxVelocity, double maxAcceleration) {
        amper = Amper.getInstance();

        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    @Override
    public void initialize() {
        amper.setConstraints(maxVelocity, maxAcceleration);
    }
}
