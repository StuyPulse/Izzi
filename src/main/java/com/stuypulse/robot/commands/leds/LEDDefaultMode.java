/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * White lost vision
 *
 * Orange speaker
 * Gold amp
 *
 * Red for should stop intake (intake ir sees smth)
 * Rainbow for when at right height for trap
 */
public class LEDDefaultMode extends Command {

    private final LEDController leds;

    private final AprilTagVision vision;
    private final Conveyor conveyor;
    private final Intake intake;
    private final Amper amper;

    public LEDDefaultMode() {
        leds = LEDController.getInstance();
        vision = AprilTagVision.getInstance();
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();
        amper = Amper.getInstance();

        addRequirements(leds);
    }

    private LEDInstruction getInstruction() {
        if (Math.abs(amper.getLiftHeight() - Lift.TRAP_SCORE_HEIGHT) < Lift.MAX_HEIGHT_ERROR)
            return LEDInstructions.TRAP;
        if (conveyor.isNoteAtShooter()) return LEDInstructions.SPEAKER;
        if (amper.hasNote()) return LEDInstructions.AMP;
        if (intake.hasNote()) return LEDInstructions.RED;
        if (RobotBase.isReal()) {
            if (vision.getOutputs().isEmpty()) return LEDInstructions.WHITE;
        }

        return LEDInstructions.DEFAULT;
    }

    @Override
    public void execute() {
        leds.runLEDInstruction(getInstruction());
    }
}
