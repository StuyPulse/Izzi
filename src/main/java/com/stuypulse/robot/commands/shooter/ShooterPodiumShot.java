package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ShooterSpeeds;

public class ShooterPodiumShot extends ShooterSetRPM{

    public ShooterPodiumShot() {
        super(new ShooterSpeeds(Settings.Shooter.PODIUM_SHOT_LEFT_RPM, Settings.Shooter.PODIUM_SHOT_RIGHT_RPM));
    }
}
