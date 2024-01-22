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


    public abstract void toShooter();
        
    
    public abstract void toAmp();
}
    