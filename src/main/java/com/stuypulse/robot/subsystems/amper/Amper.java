package com.stuypulse.robot.subsystems.amper;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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

    protected final SmartNumber targetHeight;
    protected final Mechanism2d mechanism2d;
    protected final MechanismLigament2d lift2d;

    public Amper() {
        targetHeight = new SmartNumber("Amp/Target Height", 0); // TODO: determine the default value

        mechanism2d = new Mechanism2d(3, 3);
        MechanismRoot2d root = mechanism2d.getRoot("Base", 1, 1);
        lift2d = root.append(new MechanismLigament2d(
            "Lift",
            getLiftHeight(),
            Settings.Amper.Lift.ANGLE_TO_GROUND.getDegrees(),
            2,
            new Color8Bit(Color.kBlue)
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