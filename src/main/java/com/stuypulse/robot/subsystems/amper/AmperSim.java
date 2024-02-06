package com.stuypulse.robot.subsystems.amper;

import static com.stuypulse.robot.constants.Settings.Amper.Lift.CARRIAGE_MASS;
import static com.stuypulse.robot.constants.Settings.Amper.Lift.MAX_HEIGHT;
import static com.stuypulse.robot.constants.Settings.Amper.Lift.MIN_HEIGHT;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.constants.Settings.Amper.Lift.Encoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ElevatorFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AmperSim extends Amper {

    private final ElevatorSim sim;

    private final Controller controller;

    private Optional<Double> voltageOverride;

    public AmperSim() {
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

        controller = new MotorFeedforward(Lift.Feedforward.kS, Lift.Feedforward.kV, Lift.Feedforward.kA).position()
            .add(new ElevatorFeedforward(Lift.Feedforward.kG))
            .setSetpointFilter(new MotionProfile(Lift.VEL_LIMIT, Lift.ACC_LIMIT))
                .add(new PIDController(Lift.PID.kP, Lift.PID.kI, Lift.PID.kD));

        voltageOverride = Optional.empty();
    }

    @Override
    public void setTargetHeight(double height) {
        super.setTargetHeight(height);

        voltageOverride = Optional.empty();
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
	public void stopRoller() {}

    @Override
    public void setVoltageOverride(double voltage) {
        voltageOverride = Optional.of(voltage);
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
    }

    @Override
    public void simulationPeriodic() {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        sim.update(Settings.DT);
    }
    
}