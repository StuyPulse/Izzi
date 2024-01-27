package com.stuypulse.robot.subsystems.amper;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
        if(Robot.isReal()) {
            instance = new AmperImpl();
        }
        else {
            instance = new AmperSim();
        }
    }

    public static Amper getInstance() {
        return instance;
    }

    protected final Controller liftController;
    protected final SmartNumber targetHeight;
    protected final Mechanism2d mechanism2d;
    protected final MechanismLigament2d lift2d;

    public Amper() {
        liftController = new PIDController(Lift.PID.kP, Lift.PID.kI, Lift.PID.kD).setOutputFilter(
            new MotionProfile(Lift.VEL_LIMIT, Lift.ACC_LIMIT)
        );

        targetHeight = new SmartNumber("Amper/Target Height", 0); // TODO: determine the default value

        mechanism2d = new Mechanism2d(3, 3);
        mechanism2d.getRoot("Base Origin", 1, 1).append(new MechanismLigament2d(
            "Base",
            1,
            0,
            10,
            new Color8Bit(Color.kOrange)
        ));
        lift2d = mechanism2d.getRoot("Lift Origin", 1.5, 1).append(new MechanismLigament2d(
            "Lift",
            Settings.Amper.Lift.REST_HEIGHT,
            Settings.Amper.Lift.ANGLE_TO_GROUND.getDegrees(),
            10,
            new Color8Bit(Color.kAqua)
        ));
        
        SmartDashboard.putData("Lift Mechanism", mechanism2d);
    }

    public void setTargetHeight(double height) {
        targetHeight.set(SLMath.clamp(height, Settings.Amper.Lift.MIN_HEIGHT, Settings.Amper.Lift.MAX_HEIGHT));
    }

    public void initMechanism2d() {
    }
    
    public abstract boolean hasNote();

    public abstract void score();
    public abstract void intake();
    public abstract void stopRoller();
    
    public abstract boolean liftAtBottom();
    public abstract double getLiftHeight();
    public abstract void stopLift();

    public abstract boolean touchingAmp();

    @Override
    public void periodic() {
    }
}