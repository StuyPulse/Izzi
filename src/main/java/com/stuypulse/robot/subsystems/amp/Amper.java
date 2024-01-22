package com.stuypulse.robot.subsystems.amp;

import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
AMP:
1 motor
1 limit switch
IR Sensor

LIFT:
1 motor
1 encoder
Bottom (shooter) limit switch
*/

public abstract class Amper extends SubsystemBase {

    private static Amper instance;

    static {
        instance = new AmperImpl(Ports.Amp.SCORE);
    }

    public Amper getInstance() {
        return instance;
    }

    public abstract boolean hasNote();
    public abstract void acquire();
    public abstract void deacquire();
    public abstract void lift(double height);
    public abstract void stopLift();
}