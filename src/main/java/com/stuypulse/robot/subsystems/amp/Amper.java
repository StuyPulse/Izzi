package com.stuypulse.robot.subsystems.amp;

import com.stuypulse.robot.constants.Settings.Amp.LiftPID;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartBoolean;
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
    public final Controller liftController;

    public Amper() {
        liftController = new PIDController(LiftPID.kP, LiftPID.kI, LiftPID.kD);
        targetHeight = new SmartNumber("Amp/Target Height", 0); // TODO: determine the default value
    }

    public void setTargetHeight(double height) {
        targetHeight.set(height);
    }

    public abstract boolean hasNote();

    public abstract void intake();
    public abstract void score();
    
    public abstract boolean liftAtBottom();
    public abstract boolean liftAtTop();

    public abstract boolean touchingAmp();

    public abstract void stopLift();

}