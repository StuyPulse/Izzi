/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.conveyor.ConveyorMode;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ConveyorToAmp extends ConveyorSetMode {
    public static Command withCheckLift() {
        return AmperToHeight.untilDone(Settings.Amper.Lift.MIN_HEIGHT)
            .andThen(new ConveyorToAmp());
    }

    public ConveyorToAmp() {
        super(ConveyorMode.TO_AMP);
    }

    @Override
    public void initialize() {
        super.initialize();

        Shooter.getInstance().setTargetSpeeds(Settings.Shooter.HANDOFF);
    }

}
