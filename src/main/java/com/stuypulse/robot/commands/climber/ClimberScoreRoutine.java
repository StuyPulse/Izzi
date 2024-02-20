/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.commands.amper.AmperScore;
import com.stuypulse.robot.commands.amper.AmperSetLiftConstraints;
import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimberScoreRoutine extends SequentialCommandGroup {

    private final double DELAY_LIFT_SECONDS = 0.5;
    private final double AMPER_MAX_VELOCITY = 1.0;
    private final double AMPER_MAX_ACCELERATION = 1.0;
    private final double AMPER_SCORE_SECONDS = 2;

    public ClimberScoreRoutine() {
        addCommands(
            // lower climber and raise lift (after delay) simultaneously
            ClimberToHeight.untilDone(Settings.Climber.MIN_HEIGHT)
                .alongWith(
                    new SequentialCommandGroup(
                    new WaitCommand(DELAY_LIFT_SECONDS),
                    // slow down lift
                    new AmperSetLiftConstraints(AMPER_MAX_VELOCITY, AMPER_MAX_ACCELERATION),
                    AmperToHeight.untilDone(Lift.TRAP_SCORE_HEIGHT),
                    new AmperSetLiftConstraints()
                    )),
            AmperScore.forSeconds(AMPER_SCORE_SECONDS)
        );
    }
}
