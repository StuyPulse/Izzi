package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;

public class ShooterPodiumShot extends ShooterSetRPM{

    public ShooterPodiumShot() {
        super(Settings.Shooter.PODIUM_SHOT_LEFT_RPM, Settings.Shooter.PODIUM_SHOT_RIGHT_RPM);
    }
}
