package com.stuypulse.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberImpl extends Climber {
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private final DigitalInput topRightLimit;
    private final DigitalInput topLeftLimit;
    private final DigitalInput bottomRightLimit;
    private final DigitalInput bottomLeftLimit;

    protected ClimberImpl() {
        rightMotor = new CANSparkMax(Ports.Climber.RIGHT_MOTOR, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Ports.Climber.LEFT_MOTOR, MotorType.kBrushless);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        rightEncoder.setPositionConversionFactor(Settings.Climber.Encoder.POSITION_CONVERSION);
        leftEncoder.setPositionConversionFactor(Settings.Climber.Encoder.POSITION_CONVERSION);

        rightEncoder.setVelocityConversionFactor(Settings.Climber.Encoder.VELOCITY_CONVERSION);
        leftEncoder.setVelocityConversionFactor(Settings.Climber.Encoder.VELOCITY_CONVERSION);

        topRightLimit = new DigitalInput(Ports.Climber.TOP_RIGHT_LIMIT);
        topLeftLimit = new DigitalInput(Ports.Climber.TOP_LEFT_LIMIT);
        bottomRightLimit = new DigitalInput(Ports.Climber.BOTTOM_RIGHT_LIMIT);
        bottomLeftLimit = new DigitalInput(Ports.Climber.BOTTOM_LEFT_LIMIT);

        Motors.Climber.LEFT_MOTOR.configure(leftMotor);
        Motors.Climber.RIGHT_MOTOR.configure(rightMotor);
    }

	@Override
	public double getHeight() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }

    @Override
    public double getVelocity() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
    }

    @Override
    public boolean atTop() {
        return !topRightLimit.get() || !topLeftLimit.get();
    }

    @Override
    public boolean atBottom() {
        return !bottomRightLimit.get() || !bottomLeftLimit.get();
    }

    public void setVoltage(double voltage) {
        if (atTop() && voltage > 0) {
            DriverStation.reportWarning("Climber Top Limit Reached", false);
            voltage = 0.0;
        } else if (atBottom() && voltage < 0) {
            DriverStation.reportWarning("Climber Bottom Limit Reached", false);
            voltage = 0.0;
        }

        rightMotor.setVoltage(voltage);
        leftMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (Math.abs(getHeight() - getTargetHeight()) < Settings.Climber.Encoder.THRESHOLD) {
            setVoltage(0.0);
        } else if (getHeight() > getTargetHeight()) {
            setVoltage(-Settings.Climber.Encoder.VOLTAGE);
        } else {
            setVoltage(Settings.Climber.Encoder.VOLTAGE);
        }
        
        SmartDashboard.putNumber("Climber/Target Height", getTargetHeight());
        SmartDashboard.putNumber("Climber/Height", getHeight());
        
        SmartDashboard.putNumber("Climber/Velocity", getVelocity());

        if (atBottom()) {
            leftEncoder.setPosition(Settings.Climber.MIN_HEIGHT);
            rightEncoder.setPosition(Settings.Climber.MIN_HEIGHT);
        }
    }
}