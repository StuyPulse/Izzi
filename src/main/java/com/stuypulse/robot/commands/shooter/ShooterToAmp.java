package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;

public class ShooterToAmp extends ShooterSetRPM {
    
    public ShooterToAmp() {
        super(Settings.Shooter.AMP_LEFT_RPM, Settings.Shooter.AMP_RIGHT_RPM);
    }
}
