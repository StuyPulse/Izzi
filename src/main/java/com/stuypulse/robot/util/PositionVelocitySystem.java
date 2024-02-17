/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class PositionVelocitySystem {

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
    public static LinearSystem<N2, N1, N2> getPositionVelocitySystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("kV must greater than zero");
        }
        if (kA <= 0.0) {
            throw new IllegalArgumentException("kA must be greater than zero");
        }

        return new LinearSystem<N2, N1, N2>(
            MatBuilder.fill(Nat.N2(), Nat.N2(), 0.0, 1.0, 0.0, -kV / kA),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 1.0 / kA),
            MatBuilder.fill(Nat.N2(), Nat.N2(), 1.0, 0.0, 0.0, 1.0),
            MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0, 0.0));
    }

    /**
     * Gets the simulation of the system.
     *
     * @return A LinearSystemSim representing the simulation of the system.
     */
    public static LinearSystemSim<N2, N1, N2> getPositionVelocitySim(double kV, double kA) {
        return new LinearSystemSim<N2, N1, N2>(getPositionVelocitySystem(kV, kA));
    }
}
