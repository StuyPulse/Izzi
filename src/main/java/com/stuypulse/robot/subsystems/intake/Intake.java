/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.util.IntakeVisualizer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

    private static final Intake instance;

    static {
        if (RobotBase.isReal()) {
            instance = new IntakeImpl();
        } else {
            instance = new IntakeSim();
        }
    }

    public static Intake getInstance() {
        return instance;
    }

    private final IntakeVisualizer intakeVisualizer;

    protected Intake() {
        intakeVisualizer = new IntakeVisualizer();
    }

    public abstract void acquire();

    public abstract void deacquire();

    public abstract void stop();

    public abstract boolean hasNote();

    public abstract double getIntakeRollerSpeed();

    @Override
    public void periodic() {
        Conveyor conveyor = Conveyor.getInstance();
        intakeVisualizer.update(
            hasNote(),
            conveyor.isNoteAtShooter(),
            Amper.getInstance().hasNote(),
            getIntakeRollerSpeed(),
            conveyor.getGandalfSpeed(),
            conveyor.getFeederSpeed());
    }
}
