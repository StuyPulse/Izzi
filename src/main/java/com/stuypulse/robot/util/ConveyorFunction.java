package com.stuypulse.robot.util;

@FunctionalInterface
public interface ConveyorFunction<T, U, V> {
    
    public boolean accept(T t, U u, V v);

}
