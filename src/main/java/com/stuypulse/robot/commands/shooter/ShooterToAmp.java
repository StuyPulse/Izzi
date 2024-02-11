package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ShooterSpeeds;

public class ShooterToAmp extends ShooterSetRPM {
    
    public ShooterToAmp() {
        super(new ShooterSpeeds(Settings.Shooter.AMP_LEFT_RPM, Settings.Shooter.AMP_RIGHT_RPM));
    }
}
