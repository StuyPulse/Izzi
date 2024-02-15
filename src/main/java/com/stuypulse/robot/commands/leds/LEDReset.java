/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.subsystems.leds.LEDController;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDReset extends InstantCommand {

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private final LEDController ledController;

    public LEDReset() {
        this.ledController = LEDController.getInstance();

        addRequirements(ledController);
    }

    @Override
    public void initialize() {
        ledController.runLEDInstruction(LEDInstructions.DEFAULT);
    }
}
