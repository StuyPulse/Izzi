package com.stuypulse.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Settings;

public class SimClimber extends Climber {
    private final ElevatorSim sim;

    public SimClimber() {
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
    }

    @Override
    public double getHeight() {
        return sim.getPositionMeters();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public boolean atTop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'atTop'");
    }

    @Override
    public boolean atBottom() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'atBottom'");
    }

    public void setVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }

    @Override
    public void simulationPeriodic() {
        super.periodic();

        sim.update(Settings.DT);

        if (driveVoltage.isPresent()) {
            setVoltage(driveVoltage.get());
        } else {
            
            if (Math.abs(getHeight() - getTargetHeight()) < Settings.Climber.Encoder.THRESHOLD) {
                setVoltage(0.0);
            } else if (getHeight() > getTargetHeight()) {
                setVoltage(-Settings.Climber.Encoder.VOLTAGE);
            } else {
                setVoltage(Settings.Climber.Encoder.VOLTAGE);
            }

        }

        if (getHeight() >= Settings.Climber.MIN_HEIGHT) {
            climberVisualizer.setHeight(getHeight());
        }

        SmartDashboard.putNumber("Climber/Height", getHeight());
        SmartDashboard.putNumber("Climber/Velocity", getVelocity());
    }
}
