/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final Odometry odometry;
    private final Intake intake;
    private final Amper amper;

    public LEDDefaultMode() {
        leds = LEDController.getInstance();
        odometry = Odometry.getInstance();
        intake = Intake.getInstance();
        amper = Amper.getInstance();

        addRequirements(leds);
    }

    private LEDInstruction getInstruction() {
        if (amper.getTargetHeight() == Lift.TRAP_SCORE_HEIGHT
                && amper.isAtTargetHeight(0.15))
            return LEDInstructions.GREEN;
        
        if (DriverStation.isAutonomousEnabled() && Math.abs(Conveyor.getInstance().getGandalfSpeed()) > 0) {
            return LEDInstructions.PURPLE;
        }

        if (DriverStation.isAutonomousEnabled() && SmartDashboard.getBoolean("AutonAlignment", false)) {
            return LEDInstructions.GREEN;
        }

        if (intake.hasNote()) {
            return LEDInstructions.CONTAINS_NOTE;
        }

        if (odometry.getPose().getX() > Field.LENGTH / 2.0) {
            return LEDInstructions.ATTENTION;
        }
        
        return LEDInstructions.DEFAULT;
    }

    @Override
    public void execute() {
        leds.runLEDInstruction(getInstruction());
    }
}