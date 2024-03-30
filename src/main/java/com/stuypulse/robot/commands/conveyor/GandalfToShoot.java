/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.ConveyorMode;

public class GandalfToShoot extends ConveyorSetMode {

    public GandalfToShoot() {
        super(ConveyorMode.GANDALF_SHOOT);
    }

}
