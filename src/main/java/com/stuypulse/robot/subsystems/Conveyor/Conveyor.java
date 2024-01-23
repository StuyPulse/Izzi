package com.stuypulse.robot.subsystems.Conveyor;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * field:
 * - Motor (cansparkmax, kraken?)
 * - NO sensors
 * - set mode
 * 
 * Methods
 * - toAmp()
 * - toShooter()
 */

public abstract class Conveyor extends SubsystemBase {
    public enum Direction {
        NONE, SHOOTER, AMP
    }

    private static final Conveyor instance;

    static{
        if(RobotBase.isSimulation()){
            // create a conveyor sim
            instance = null;
        } 
        else if(Settings.ROBOT == Settings.Robot.BIG_WANG){
            instance = new ConveyorImpl();
        }
        else {
            instance = null;//should be conveyor sim
        }
    }

    public static Conveyor getInstance(){
        return instance;
    };

    public abstract Conveyor.Direction getTarget();
    public abstract void setTarget(Conveyor.Direction target);

    public abstract void gandalfToShooter();
    public abstract void gandalfToAmp();
    public abstract void gandalfStop();

    public abstract void feederForward();
    public abstract void feederStop();
}
    