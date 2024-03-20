/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkBase;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {

    public enum StatusFrame {
        APPLIED_OUTPUT_FAULTS,
        MOTOR_VEL_VOLTS_AMPS,
        MOTOR_POSITION,
        ANALOG_SENSOR,
        ALTERNATE_ENCODER,
        ABS_ENCODER_POSIITION,
        ABS_ENCODER_VELOCITY
    }

    public static void disableStatusFrames(CANSparkBase motor, StatusFrame... ids) {
        final int kDisableStatusFrame = 500;

        for (StatusFrame id : ids) {
            motor.setPeriodicFramePeriod(PeriodicFrame.fromId(id.ordinal()), kDisableStatusFrame);
        }
    }

    /** Classes to store all of the values a motor needs */
    public interface Amper {
        CANSparkConfig LIFT_MOTOR = new CANSparkConfig(true, IdleMode.kBrake, 80);
        CANSparkConfig SCORE_MOTOR = new CANSparkConfig(true, IdleMode.kBrake, 500, 0.1);
    }

    public interface Swerve {
        CANSparkConfig DRIVE_CONFIG = new CANSparkConfig(true, IdleMode.kBrake, 60);
        CANSparkConfig TURN_CONFIG = new CANSparkConfig(false, IdleMode.kBrake, 80);
    }

    public interface Intake {
        CANSparkConfig MOTOR_CONFIG = new CANSparkConfig(false, IdleMode.kBrake, 500, 0.1);
    }

    public interface Shooter {
        CANSparkConfig LEFT_SHOOTER = new CANSparkConfig(false, IdleMode.kCoast, 500, 0.5);
        CANSparkConfig RIGHT_SHOOTER = new CANSparkConfig(true, IdleMode.kCoast, 500, 0.5);
    }

    public interface Conveyor {
        CANSparkConfig GANDALF_MOTOR = new CANSparkConfig(true, IdleMode.kBrake,500, 0.1);
        CANSparkConfig SHOOTER_FEEDER_MOTOR = new CANSparkConfig(false, IdleMode.kBrake, 500, 0.1);
    }

    public interface Climber {
        CANSparkConfig LEFT_MOTOR = new CANSparkConfig(true, IdleMode.kBrake, 80, 0.1);
        CANSparkConfig RIGHT_MOTOR = new CANSparkConfig(false, IdleMode.kBrake, 80, 0.1);
    }

    public static class CANSparkConfig {
        public final boolean INVERTED;
        public final IdleMode IDLE_MODE;
        public final int CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public CANSparkConfig(
                boolean inverted,
                IdleMode idleMode,
                int currentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.IDLE_MODE = idleMode;
            this.CURRENT_LIMIT_AMPS = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public CANSparkConfig(boolean inverted, IdleMode idleMode, int currentLimitAmps) {
            this(inverted, idleMode, currentLimitAmps, 0.0);
        }

        public CANSparkConfig(boolean inverted, IdleMode idleMode) {
            this(inverted, idleMode, 500);
        }

        public void configure(CANSparkBase motor) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.burnFlash();
        }
    }
}
