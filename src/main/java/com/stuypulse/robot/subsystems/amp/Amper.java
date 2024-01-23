package com.stuypulse.robot.subsystems.amp;

import com.stuypulse.robot.constants.Settings.Amp.Lift;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

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
        instance = new AmperImpl();
    }

    public Amper getInstance() {
        return instance;
    }

    public final SmartNumber targetHeight;
    public final PIDController liftController;

    public Amper() {
        liftController = new PIDController(Lift.kP, Lift.kI, Lift.kD);
        targetHeight = new SmartNumber("Amp/Target Height", 0); // TODO: determine the default value
    }

    public abstract boolean hasNote();
    public abstract void acquire();
    public abstract void deacquire();
    public abstract void setTargetHeight(double height);
    public abstract void stopLift();
}