/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*-
 * Contains:
 *      - forceSetLED() : sets color of LEDs for short time
 *      - getDefaultColor() : determines LED color if it is not set
 *      - periodic() : sets LED color to default color if in teleop
 */

public class LEDController extends SubsystemBase {
    private static LEDController instance;

    static {
        if (Robot.ROBOT == RobotType.JIM) {
            instance = new LEDController(0, 55);
        } else {
            instance = new LEDController(Ports.LEDController.PORT, Settings.LED.LED_LENGTH);
        }
    }

    public static LEDController getInstance() {
        return instance;
    }

    private AddressableLED leds;
    private AddressableLEDBuffer ledsBuffer;

    protected LEDController(int port, int length) {
        leds = new AddressableLED(port);
        ledsBuffer = new AddressableLEDBuffer(length);

        leds.setLength(ledsBuffer.getLength());
        leds.setData(ledsBuffer);
        leds.start();

        runLEDInstruction(LEDInstructions.DEFAULT);

        SmartDashboard.putData(instance);
    }

    public void runLEDInstruction(LEDInstruction instruction) {
        instruction.setLED(ledsBuffer);
        leds.setData(ledsBuffer);
    }
}
