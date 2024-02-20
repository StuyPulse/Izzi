/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings.Amper.Lift;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmperScoreTrapRoutine extends SequentialCommandGroup {

    public AmperScoreTrapRoutine() {
        addCommands(
            new AmperToHeight(Lift.TRAP_SCORE_HEIGHT),
            new AmperScore()
        );
    }
}
