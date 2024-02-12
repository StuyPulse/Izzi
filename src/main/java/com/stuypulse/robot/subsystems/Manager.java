package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manager extends SubsystemBase {
    // singleton
    private static Manager instance;

    static {
        instance = new Manager();
    }

    public static Manager getInstance() {
        return instance;
    }

    // object youre scoring at
    public enum ScoreLocation {
        SPEAKER,
        AMP,
        TRAP
    }
    
    private ScoreLocation location;

    private Manager() {
        location = ScoreLocation.SPEAKER;
    }

    public ScoreLocation getScoreLocation() {
        return this.location;
    }

    public void setScoreLocation(ScoreLocation location) {
        this.location = location;
    }

    @Override
    public void periodic() {
        if (Amper.getInstance().getTargetHeight() == Settings.Amper.Lift.AMP_SCORE_HEIGHT.get()) {
            setScoreLocation(ScoreLocation.AMP);
        }
        if (Amper.getInstance().getTargetHeight() == Settings.Amper.Lift.TRAP_SCORE_HEIGHT.get()) {
            setScoreLocation(ScoreLocation.TRAP);
        } 
        if (Conveyor.getInstance().isNoteAtShooter()) {
            setScoreLocation(ScoreLocation.SPEAKER);
        }
        SmartDashboard.putString("Score Location", location.name());
    }
}
