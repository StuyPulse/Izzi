/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

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
    
    /** Classes to store all of the values a motor needs */
  
    public interface Amper {
        CANSparkMaxConfig LIFT_MOTOR = new CANSparkMaxConfig(false, IdleMode.kBrake);
        CANSparkMaxConfig SCORE_MOTOR = new CANSparkMaxConfig(false, IdleMode.kBrake);
    }
  
    public interface Swerve {
        public CANSparkMaxConfig DRIVE_CONFIG = new CANSparkMaxConfig(false, IdleMode.kBrake);
        public CANSparkMaxConfig TURN_CONFIG = new CANSparkMaxConfig(false, IdleMode.kBrake);
    }

    public interface Intake {
        CANSparkMaxConfig MOTOR_CONFIG = new CANSparkMaxConfig(false, IdleMode.kBrake);
    }
  
    public interface Shooter {
        CANSparkMaxConfig LEFT_SHOOTER = new CANSparkMaxConfig(false,IdleMode.kCoast);
        CANSparkMaxConfig RIGHT_SHOOTER = new CANSparkMaxConfig(false,IdleMode.kCoast);      
    }
  
    public interface Conveyor {
        CANSparkMaxConfig GANDALF_MOTOR = new CANSparkMaxConfig(false, IdleMode.kBrake);
        CANSparkMaxConfig FEEDER_MOTOR = new CANSparkMaxConfig(false, IdleMode.kBrake);
    }

    public interface Climber {
        CANSparkMaxConfig LEFT_MOTOR = new CANSparkMaxConfig(false, IdleMode.kBrake);
        CANSparkMaxConfig RIGHT_MOTOR = new CANSparkMaxConfig(false, IdleMode.kBrake);
    }

    public static class CANSparkMaxConfig {
        public final boolean INVERTED;
        public final IdleMode IDLE_MODE;
        public final int CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public CANSparkMaxConfig(
                boolean inverted,
                IdleMode idleMode,
                int currentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.IDLE_MODE = idleMode;
            this.CURRENT_LIMIT_AMPS = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode, int currentLimitAmps) {
            this(inverted, idleMode, currentLimitAmps, 0.05);
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode) {
            this(inverted, idleMode, 80);
        }

        public void configure(CANSparkMax motor) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.burnFlash();
         }
          
     }

}
