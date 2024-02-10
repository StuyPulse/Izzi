package com.stuypulse.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;

public class ClimberSim extends Climber {
    
    private final ElevatorSim sim;

    private Optional<Double> voltageOverride;

    public ClimberSim() {
        sim = new ElevatorSim(
            DCMotor.getNEO(2), 
            Settings.Climber.Encoder.GEAR_RATIO, 
            Settings.Climber.MASS, 
            Settings.Climber.DRUM_RADIUS, 
            Settings.Climber.MIN_HEIGHT, 
            Settings.Climber.MAX_HEIGHT, 
            true, 
            Settings.Climber.MIN_HEIGHT
        );
        
        voltageOverride = Optional.empty();
    }

    @Override
    public void setTargetHeight(double height) {
        super.setTargetHeight(height);

        voltageOverride = Optional.empty();
    }

    @Override
    public double getHeight() {
        return sim.getPositionMeters();
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

    /*** LIMITS ***/

    @Override
    public boolean atTop() {
        return sim.hasHitUpperLimit();
    }

    @Override
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
        } else {
            if (isAtTargetHeight(Settings.Climber.BangBang.THRESHOLD)) {
                setVoltage(0.0);
            } else if (getHeight() > getTargetHeight()) {
                setVoltage(-Settings.Climber.BangBang.CONTROLLER_VOLTAGE);
            } else {
                setVoltage(+Settings.Climber.BangBang.CONTROLLER_VOLTAGE);
            }
        }

        SmartDashboard.putNumber("Climber/Height", getHeight());
        SmartDashboard.putNumber("Climber/Velocity", getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        sim.update(Settings.DT);
    }
}
