package com.stuypulse.robot.util;

import com.stuypulse.stuylib.streams.numbers.filters.IFilter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StupidFilter implements IFilter {

    private double lastRealValue;

    public StupidFilter() {
        lastRealValue = 0;
    }

    @Override
    public double get(double value) {
        if (Double.isNaN(value) || Math.abs(value) > 1e9) {
            SmartDashboard.putNumber("NaN Count", SmartDashboard.getNumber("NaN Count", 0) + 1);
    
            return lastRealValue;
        }

        lastRealValue = value;

        return lastRealValue;
    }

}
