package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimShooter extends Shooter {
    private FlywheelSim leftWheel;
    private FlywheelSim rightWheel;
    
    public SimShooter() {
        leftWheel = new FlywheelSim(DCMotor.getNEO(1), 1, Settings.Shooter.MOMENT_OF_INERTIA);
        rightWheel = new FlywheelSim(DCMotor.getNEO(1), 1, Settings.Shooter.MOMENT_OF_INERTIA);
    }
    
    public void stop() {
        leftWheel.setInputVoltage(0);
        rightWheel.setInputVoltage(0);
    }
    
    public double getLeftShooterRPM() {
        return leftWheel.getAngularVelocityRPM();
    }

    public double getRightShooterRPM() {
        return rightWheel.getAngularVelocityRPM();
    }

    public void setRightMotorVoltageImpl(double voltage) {
        rightWheel.setInputVoltage(voltage);
    }

    public void setLeftMotorVoltageImpl(double voltage) {
        leftWheel.setInputVoltage(voltage);
    }

    @Override
    public void simulationPeriodic() {
        leftWheel.update(Settings.DT);
        rightWheel.update(Settings.DT);

        SmartDashboard.putNumber("Shooter/Left Target RPM",getLeftTargetRPM());
        SmartDashboard.putNumber("Shooter/Right Target RPM", getRightTargetRPM());
        
        SmartDashboard.putNumber("Shooter/Right RPM", getRightShooterRPM());
        SmartDashboard.putNumber("Shooter/Left RPM", getLeftShooterRPM());
    }
    
}
