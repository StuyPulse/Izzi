/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.amper.AmperScoreTrap;
import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.commands.conveyor.ConveyorToAmp;
import com.stuypulse.robot.constants.Settings.Amper.Lift;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TrapScoreRoutine extends SequentialCommandGroup {

    private final double SCORE_WAIT = 1.5;

    public TrapScoreRoutine() {
        addCommands(
            new ConveyorToAmp(),
            AmperToHeight.untilDone(Lift.TRAP_SCORE_HEIGHT, 0.15)
            // new WaitCommand(SCORE_WAIT),
            // new AmperScoreTrap()
        );
    }
}
