package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.util.ShooterSpeeds;

public class ShooterStop extends ShooterSetRPM {    

    public ShooterStop(){
        super(new ShooterSpeeds());
    }
}
