/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings.RobotType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

    private static final Intake instance;

    static {
        if (Robot.ROBOT == RobotType.IZZI) {
            instance = new IntakeImpl();
        } else if (Robot.ROBOT == RobotType.TUMBLER) {
            instance = new TumblerIntake();
        } else {
            instance = new IntakeSim();
        }
    }

    public static Intake getInstance() {
        return instance;
    }

    public abstract void acquire();

    public abstract void deacquire();

    public abstract void stop();

    public abstract void setIdleMode(IdleMode mode);

    public abstract boolean hasNote();

    public boolean hasNotePartially() {
        return hasNote();
    }

    public abstract double getIntakeRollerSpeed();

    @Override
    public void periodic() {

    }
}
