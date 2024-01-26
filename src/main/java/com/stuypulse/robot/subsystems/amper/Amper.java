package com.stuypulse.robot.subsystems.amper;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
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

    public static Amper getInstance() {
        return instance;
    }

    public final SmartNumber targetHeight;
    public final Controller liftController;

    public Amper() {
        liftController = new PIDController(Lift.PID.kP, Lift.PID.kI, Lift.PID.kD);
        targetHeight = new SmartNumber("Amp/Target Height", 0); // TODO: determine the default value
    }

    public void setTargetHeight(double height) {
        targetHeight.set(SLMath.clamp(height, Settings.Amper.Lift.MIN_HEIGHT, Settings.Amper.Lift.MAX_HEIGHT));
    }
    
    public abstract boolean hasNote();

    public abstract void score();
    public abstract void intake();
    public abstract void stopRoller();
    
    public abstract boolean liftAtBottom();
    public abstract double getLiftHeight();
    public abstract void stopLift();
    public abstract void setLiftVoltageImpl(double voltage);

    public abstract boolean touchingAmp();

    @Override
    public void periodic() {
        setLiftVoltageImpl(liftController.update(targetHeight.get(), getLiftHeight()));
    }

}