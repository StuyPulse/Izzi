package com.stuypulse.robot.subsystems.amper;

import static com.stuypulse.robot.constants.Settings.Amper.Lift.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.constants.Settings.Amper.Lift.Encoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AmperSim extends Amper {

    private final ElevatorSim sim;

    public final Controller liftController;

    public AmperSim() {
        liftController = new PIDController(Lift.PID.kP, Lift.PID.kI, Lift.PID.kD);

        sim = new ElevatorSim(
            DCMotor.getNEO(1),
            Encoder.GEARING,
            CARRIAGE_MASS,
            Encoder.DRUM_RADIUS,
            MIN_HEIGHT,
            MAX_HEIGHT,
            true,
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
    public void simulationPeriodic() {
        liftController.update(targetHeight.get(), getLiftHeight());

        if (liftAtBottom() && liftController.getOutput() < 0) {
            stopLift();
        } else {
            sim.setInputVoltage(liftController.getOutput());
        }

        lift2d.setLength(getLiftHeight());

        sim.update(Settings.DT);

        SmartDashboard.putNumber("Amper/Lift Height", getLiftHeight());
    }
    
}