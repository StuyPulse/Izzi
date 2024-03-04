package com.stuypulse.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class FilteredRelativeEncoder implements RelativeEncoder {

    private final RelativeEncoder encoder;

    private final CANSparkBase motor;

    private double lastValidPosition, lastValidVelocity;

    public FilteredRelativeEncoder(CANSparkBase motor) {
        this(motor, motor.getEncoder());
    }
    
    public FilteredRelativeEncoder(CANSparkBase motor, RelativeEncoder encoder) {
        this.encoder = encoder;
        this.motor = motor;
        this.lastValidPosition = 0.0;
        this.lastValidVelocity = 0.0;
    }

    @Override
    public double getPosition() {
        if (motor.getLastError() == REVLibError.kOk){
            lastValidPosition = encoder.getPosition();
        }
        return lastValidPosition;
    }

    @Override
    public double getVelocity() {
        if (motor.getLastError() == REVLibError.kOk){
            lastValidVelocity = encoder.getVelocity();
        }
        return lastValidVelocity;
    }

    @Override
    public REVLibError setPosition(double position) {
        return encoder.setPosition(position);
    }

    @Override
    public REVLibError setPositionConversionFactor(double factor) {
        return encoder.setPositionConversionFactor(factor);
    }

    @Override
    public REVLibError setVelocityConversionFactor(double factor) {
        return encoder.setVelocityConversionFactor(factor);
    }

    @Override
    public double getPositionConversionFactor() {
        return encoder.getPositionConversionFactor();
    }

    @Override
    public double getVelocityConversionFactor() {
        return encoder.getVelocityConversionFactor();
    }

    @Override
    public REVLibError setAverageDepth(int depth) {
       return encoder.setAverageDepth(depth);
    }

    @Override
    public int getAverageDepth() {
        return encoder.getAverageDepth();
    }

    @Override
    public REVLibError setMeasurementPeriod(int period_ms) {
        return encoder.setMeasurementPeriod(period_ms);
    }

    @Override
    public int getMeasurementPeriod() {
        return encoder.getMeasurementPeriod();
    }

    @Override
    public int getCountsPerRevolution() {
        return encoder.getCountsPerRevolution();
    }

    @Override
    public REVLibError setInverted(boolean inverted) {
        return encoder.setInverted(inverted);
    }

    @Override
    public boolean getInverted() {
        return encoder.getInverted();
    }
    
}    