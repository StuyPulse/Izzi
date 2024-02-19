/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.amper;

import static com.stuypulse.robot.constants.Settings.Amper.Lift.CARRIAGE_MASS;
import static com.stuypulse.robot.constants.Settings.Amper.Lift.MAX_HEIGHT;
import static com.stuypulse.robot.constants.Settings.Amper.Lift.MIN_HEIGHT;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ElevatorFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.constants.Settings.Amper.Lift.Encoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AmperSim extends Amper {

    private final ElevatorSim sim;

    private final Controller controller;

    private Optional<Double> voltageOverride;

    private final SmartNumber maxVelocity;
    private final SmartNumber maxAcceleration;
    private final SmartBoolean ampIR;

    protected AmperSim() {
        sim = new ElevatorSim(
            DCMotor.getNEO(1),
            Encoder.GEARING,
            CARRIAGE_MASS,
            Encoder.DRUM_RADIUS,
            MIN_HEIGHT,
            MAX_HEIGHT,
            true,
            0);

        maxVelocity = new SmartNumber("Amper/Lift/Max Velocity", Lift.VEL_LIMIT);
        maxAcceleration = new SmartNumber("Amper/Lift/Max Acceleration", Lift.ACCEL_LIMIT);
        ampIR = new SmartBoolean("Amper/IR Sensor", false);

        controller = new MotorFeedforward(Lift.Feedforward.kS, Lift.Feedforward.kV, Lift.Feedforward.kA).position()
            .add(new ElevatorFeedforward(Lift.Feedforward.kG))
            .add(new PIDController(Lift.PID.kP, Lift.PID.kI, Lift.PID.kD))
            .setSetpointFilter(new MotionProfile(maxVelocity, maxAcceleration));

        voltageOverride = Optional.empty();
    }

    /*** LIFT CONTROL ***/

    @Override
    public void setTargetHeight(double height) {
        super.setTargetHeight(height);

        voltageOverride = Optional.empty();
    }

    @Override
    public boolean liftAtBottom() {
        return sim.hasHitLowerLimit();
    }

    @Override
    public boolean liftAtTop() {
        return sim.hasHitUpperLimit();
    }

    @Override
    public double getLiftHeight() {
        return sim.getPositionMeters();
    }

    @Override
    public void stopLift() {
        sim.setInputVoltage(0.0);
    }

    /*** IR SENSOR ***/

    @Override
    public boolean hasNote() {
        return ampIR.get();
    }

    @Override
    public void fromConveyor() {}

    @Override
    public void toConveyor() {}

    @Override
    public void score() {}

    @Override
    public void stopRoller() {}

    // @Override
    // public boolean touchingAmp() {
    //     return false;
    // }

    /*** LIFT CONFIG ***/

    @Override
    public void setVoltageOverride(double voltage) {
        voltageOverride = Optional.of(voltage);
    }

    @Override
    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity.set(maxVelocity);
        this.maxAcceleration.set(maxAcceleration);
    }

    @Override
    public double getNoteDistance() {
        return 0.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        controller.update(getTargetHeight(), getLiftHeight());

        double voltage = voltageOverride.orElse(controller.getOutput());

        if (liftAtBottom() && voltage < 0 || liftAtTop() && voltage > 0) {
            stopLift();
        } else {
            sim.setInputVoltage(voltage);
        }

        SmartDashboard.putNumber("Amper/Lift Current", sim.getCurrentDrawAmps());
        SmartDashboard.putNumber("Amper/Lift Height", getLiftHeight());
        SmartDashboard.putBoolean("Amper/Under Stage?", AmperToHeight.isUnderStage());
        
    }

    @Override
    public void simulationPeriodic() {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        sim.update(Settings.DT);
    }
}
