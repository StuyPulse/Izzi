package com.stuypulse.robot.subsystems.amper;

import static com.stuypulse.robot.constants.Settings.Amper.Lift.*;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AmperSim extends Amper {

    private final ElevatorSim sim;

    private double liftHeight;
    private double liftVelocity;

    public AmperSim() {
        liftHeight = 0.0;
        liftVelocity = 0.0;

        sim = new ElevatorSim(
            DCMotor.getNEO(1),
            GEARING,
            CARRIAGE_MASS,
            DRUM_RADIUS,
            MIN_HEIGHT,
            MAX_HEIGHT,
            true,
            liftHeight
        );
    }

    @Override
    public boolean hasNote() {
        return false;
    }

    @Override
    public void intake() {}

    @Override
    public void score() {}

    @Override
    public boolean liftAtBottom() {
        return sim.hasHitLowerLimit();
    }

    @Override
    public boolean liftAtTop() {
        return sim.hasHitUpperLimit();
    }

    @Override
    public boolean touchingAmp() {
        return false;
    }

    @Override
    public void stopLift() {
        sim.setInputVoltage(0.0);
    }

    @Override
    public double getLiftHeight() {
        return sim.getPositionMeters();
    }

    @Override 
    public void setLiftVoltageImpl() {
        
    }

	@Override
	public void stopRoller() {}

    @Override
    public void simulationPeriodic() {
        sim.update(Settings.DT);

        liftHeight = sim.getPositionMeters();
        liftVelocity = sim.getVelocityMetersPerSecond();

        SmartDashboard.putNumber("AmperSim/Height", liftHeight);
        SmartDashboard.putNumber("AmperSim/Velocity", liftVelocity);
    }


    
}

        