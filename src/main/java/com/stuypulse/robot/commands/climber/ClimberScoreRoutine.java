package com.stuypulse.robot.commands.climber;
import com.stuypulse.robot.commands.amper.AmperScore;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimberScoreRoutine extends SequentialCommandGroup {
    public ClimberScoreRoutine() {
        addCommands(
            ClimberToHeight.untilDone(Settings.Climber.MIN_HEIGHT),
            new AmperScore()
        );
    }   
}