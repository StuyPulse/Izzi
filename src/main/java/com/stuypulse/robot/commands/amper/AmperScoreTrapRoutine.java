package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings.Amper.Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmperScoreTrapRoutine extends SequentialCommandGroup {
    
    public AmperScoreTrapRoutine() {
        addCommands(
            new AmperToHeight(Score.TRAP_SCORE_HEIGHT.get()),
            new AmperToHeight(Score.TRAP_SCORE_HEIGHT.get()),
            new AmperScore()
        ); 
    }
    
}