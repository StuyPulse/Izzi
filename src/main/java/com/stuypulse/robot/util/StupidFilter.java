package com.stuypulse.robot.util;

import com.stuypulse.stuylib.streams.numbers.filters.IFilter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StupidFilter implements IFilter {

    private final String name;

    private double lastRealValue;

    private int nanCount;

    public StupidFilter(String name) {
        this.name = name;

        lastRealValue = 0;

        nanCount = 0;
    }

    @Override
    public double get(double value) {
        if (Double.isNaN(value) || Math.abs(value) > 1e9) {
            SmartDashboard.putNumber(name + " NaN Count", ++nanCount);
    
            return lastRealValue;
        }

        lastRealValue = value;

        return lastRealValue;
    }

}
