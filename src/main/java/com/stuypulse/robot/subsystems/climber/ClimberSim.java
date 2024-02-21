/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class ClimberSim extends Climber {

    private final ElevatorSim sim;

    private Optional<Double> voltageOverride;

    protected ClimberSim() {
        sim = new ElevatorSim(
            DCMotor.getNEO(2),
            Settings.Climber.Encoder.GEAR_RATIO,
            Settings.Climber.MASS,
            Settings.Climber.Encoder.SPROCKET_RADIUS,
            Settings.Climber.MIN_HEIGHT,
            Settings.Climber.MAX_HEIGHT,
            true,
            Settings.Climber.MIN_HEIGHT);

        voltageOverride = Optional.empty();
    }

    @Override
    public double getLeftHeight() {
        return sim.getPositionMeters();
    }

    @Override
    public double getRightHeight() {
        return sim.getPositionMeters();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public void toTop() {
        setVoltage(+Settings.Climber.Control.UP_VOLTAGE);
    }

    @Override
    public void toBottom() {
        setVoltage(-Settings.Climber.Control.DOWN_VOLTAGE);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    /*** LIMITS ***/

    public boolean atTop() {
        return sim.hasHitUpperLimit();
    }

    public boolean atBottom() {
        return sim.hasHitLowerLimit();
    }

    @Override
    public void setVoltageOverride(double voltage) {
        voltageOverride = Optional.of(voltage);
    }

    private void setVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (voltageOverride.isPresent()) {
            setVoltage(voltageOverride.get());
        }

        SmartDashboard.putNumber("Climber/Height", getHeight());
        SmartDashboard.putNumber("Climber/Velocity", getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        sim.update(Settings.DT);
    }
}
