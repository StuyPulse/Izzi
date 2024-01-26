package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings.Amper.Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreTrap extends SequentialCommandGroup {
    
    public ScoreTrap() {
        addCommands(
            new LiftToHeight(Score.TRAP_SCORE_HEIGHT.get(), Score.TRAP_SCORE_THRESHOLD.get()),
            new RunRoller(true)
        ); 
    }
    
}