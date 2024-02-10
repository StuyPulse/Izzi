package com.stuypulse.robot.commands.conveyor;

public class ConveyorOuttake extends ConveyorRecall {
    @Override
    public boolean isFinished() {
        return false;
    }
}
