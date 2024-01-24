package com.stuypulse.robot.util;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class PositionVelocitySystem {
    private double kV;
    private double kA;

    private LinearSystem<N2, N1, N2> system;
    private LinearSystemSim<N2, N1, N2> sim;

    /**
     * Create a state-space model for a 1 DOF position system from its kV (volts/(unit/sec)) and kA (volts/(unit/sec²). 
     * <li> States: [position, velocity]ᵀ 
     * <li> Inputs  [voltage]
     * <li> Outputs [position, velocity]
     * 
     * <p>The distance unit MUST either meters or radians.
     * <p>The parameters provided by the user are from this feedforward model:
     * <p>u = K_v v + K_a a
     *
     * @param kV The velocity gain, in volts/(unit/sec)
     * @param kA The acceleration gain, in volts/(unit/sec²)
     * @throws IllegalArgumentException if kV &lt;= 0 or kA &lt;= 0.
     */
    public PositionVelocitySystem(SmartNumber kV, SmartNumber kA) {
        if (kV.get() <= 0.0) {
            throw new IllegalArgumentException("kV must greater than zero");
        }

        if (kA.get() <= 0.0) {
            throw new IllegalArgumentException("kA must be greater than zero");
        }

        this.kV = kV.get();
        this.kA = kA.get();

        this.system = new LinearSystem<N2, N1, N2> (
            MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0, 0.0, -this.kV / this.kA),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 1.0 / this.kA),
            MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0)
        );

        this.sim = new LinearSystemSim<N2, N1, N2>(this.system);
    }

    /**
     * Gets the linear system.
     * @return A LinearSystem representing the given characterized constants.
     */
    public LinearSystem<N2, N1, N2> getSystem() {
        return this.system;
    }

    /**
     * Gets the simulation of the system.
     * @return A LinearSystemSim representing the simulation of the system.
     */
    public LinearSystemSim <N2, N1, N2> getSim() {
        return this.sim;
    }
}
