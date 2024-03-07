/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.conveyor;

import com.revrobotics.CANSparkBase.IdleMode;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings.RobotType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * field:
 * - Motor (cansparkmax, kraken?)
 * - shooter IR sensor
 * - set mode
 *
 * Methods
 * - toAmp()
 * - toShooter()
 */

public abstract class Conveyor extends SubsystemBase {

    private static final Conveyor instance;

    static {
        if (Robot.ROBOT == RobotType.IZZI) {
            instance = new ConveyorImpl();
        } else {
            instance = new ConveyorSim();
        }
    }

    public static Conveyor getInstance() {
        return instance;
    }

    public abstract double getGandalfSpeed();

    public abstract void toShooter();

    public abstract void toAmp();

    public abstract void setIdleMode(IdleMode mode);

    public abstract void stop();

}
