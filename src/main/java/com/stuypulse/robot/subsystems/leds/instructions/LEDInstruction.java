/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds.instructions;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface LEDInstruction {
    void setLED(AddressableLEDBuffer ledsBuffer);
}
