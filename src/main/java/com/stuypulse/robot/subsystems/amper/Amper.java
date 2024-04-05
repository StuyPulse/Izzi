/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.amper;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.robot.constants.Settings.Amper.Lift;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.Odometry;

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

    private static final Amper instance;

    static {
        if (Robot.ROBOT == RobotType.IZZI) {
            instance = new AmperImpl();
        } else {
            instance = new AmperSim();
        }
    }

    public static Amper getInstance() {
        return instance;
    }

    private final SmartNumber targetHeight;
    private double minHeight;

    private final Mechanism2d mechanism2d;
    private final MechanismLigament2d lift2d;

    public Amper() {
        targetHeight = new SmartNumber("Amper/Target Height", 0); // TODO: determine the default value
        minHeight = Settings.Amper.Lift.MIN_HEIGHT;
        
        mechanism2d = new Mechanism2d(3, 3);
        mechanism2d.getRoot("Base Origin", 1, 1).append(new MechanismLigament2d(
            "Base",
            1,
            0,
            10,
            new Color8Bit(Color.kOrange)));

        lift2d = mechanism2d.getRoot("Lift Origin", 1.5, 1).append(new MechanismLigament2d(
            "Lift",
            Settings.Amper.Lift.VISUALIZATION_MIN_LENGTH,
            Settings.Amper.Lift.ANGLE_TO_GROUND.getDegrees(),
            10,
            new Color8Bit(Color.kAqua)));

        SmartDashboard.putData("Visualizers/Lift", mechanism2d);
    }

    /*** LIFT CONTROL ***/

    public void setTargetHeight(double height) {
        targetHeight.set(SLMath.clamp(
            height, minHeight, Lift.MAX_HEIGHT));
    }

    public final double getTargetHeight() {
        return targetHeight.get();
    }
    
    public final boolean isAtBelowTargetHeight(double epsilonMeters) {
        return getLiftHeight() <= getTargetHeight() + epsilonMeters;
    }

    public final boolean isAtTargetHeight(double epsilonMeters) {
        return Math.abs(getTargetHeight() - getLiftHeight()) < epsilonMeters;
    }

    public final void setMinHeight(double height) {
        minHeight = SLMath.clamp(height, Lift.MIN_HEIGHT, Lift.MAX_HEIGHT);
    }

    public final double getMinHeight() {
        return minHeight;
    }

    public abstract boolean liftAtBottom();

    public abstract boolean liftAtTop();

    public abstract double getLiftHeight();

    public abstract void stopLift();

    /*** IR SENSOR ***/

    public abstract boolean hasNote();

    /*** SCORE ROLLERS ***/

    public abstract void runRoller(double speed);

    public final void amp() {
        runRoller(Settings.Amper.Score.AMP_SPEED);
    }
    
    public final void trap() {
        runRoller(Settings.Amper.Score.TRAP_SPEED);
    }

    public final void fromConveyor() {
        runRoller(Settings.Amper.Score.FROM_CONVEYOR_SPEED);
    }

    public final void toConveyor() {
        runRoller(-Settings.Amper.Score.TO_CONVEYOR_SPEED);
    }

    public abstract void stopRoller();

    public abstract double getNoteDistance();

    // public abstract boolean touchingAmp();

    /*** LIFT CONFIG ***/

    public abstract void setVoltageOverride(double voltage);

    public abstract void setConstraints(double maxVelocity, double maxAcceleration);

    public final void resetConstraints() {
        setConstraints(Lift.VEL_LIMIT, Lift.ACCEL_LIMIT);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Amper/Amp Distance", Units.metersToInches(Field.WIDTH - Odometry.getInstance().getPose().getY()));
        
        lift2d.setLength(Settings.Amper.Lift.VISUALIZATION_MIN_LENGTH + getLiftHeight());

        if (targetHeight.get() > Settings.Amper.Lift.MAX_HEIGHT)
            targetHeight.set(Settings.Amper.Lift.MAX_HEIGHT);
        
        if (targetHeight.get() < getMinHeight())
            targetHeight.set(getMinHeight());
    }
}
