package com.stuypulse.robot.subsystems.Conveyor;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * field:
 * - Motor (cansparkmax, kraken?)
 * - shooter IR sensor
 * - set mode
 * 
 * Methods
 * - toAmp()
 * - toShooter()
 */

public abstract class Conveyor extends SubsystemBase {

    private static final Conveyor instance;

    static{
        if(RobotBase.isReal()){
            instance = new ConveyorImpl();
        } else{
            instance = null;
        }
    }

    public static Conveyor getInstance(){
        return instance;
    };


    public abstract void toShooter();
        
    
    public abstract void toAmp();
}
    