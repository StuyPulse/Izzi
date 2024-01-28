package com.stuypulse.robot.subsystems.amper;

import static com.stuypulse.robot.constants.Settings.Amper.Lift.CARRIAGE_MASS;
import static com.stuypulse.robot.constants.Settings.Amper.Lift.MAX_HEIGHT;
import static com.stuypulse.robot.constants.Settings.Amper.Lift.MIN_HEIGHT;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift.Encoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AmperSim extends Amper {

    private final ElevatorSim sim;

    public AmperSim() {
        sim = new ElevatorSim(
            DCMotor.getNEO(1),
            Encoder.GEARING,
            CARRIAGE_MASS,
            Encoder.DRUM_RADIUS,
            MIN_HEIGHT,
            MAX_HEIGHT,
            false,
            0
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
	public void stopRoller() {}

    @Override
    public void periodic() {
        super.periodic();

        if (liftAtBottom() && liftController.getOutput() < 0) {
            stopLift();
        } else {
            sim.setInputVoltage(liftController.getOutput());
        }

        SmartDashboard.putNumber("Amper/Lift Height", getLiftHeight());
    }

    @Override
    public void simulationPeriodic() {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        sim.update(Settings.DT);
    }
    
}