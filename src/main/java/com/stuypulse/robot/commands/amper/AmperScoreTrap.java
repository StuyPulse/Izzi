package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings.Amper.Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmperScoreTrap extends SequentialCommandGroup {
    
    public AmperScoreTrap() {
        addCommands(
            new AmperToHeight(Score.TRAP_SCORE_HEIGHT.get()),
            new AmperWaitToHeight(Score.TRAP_SCORE_HEIGHT.get()),
            new AmperScore()
        ); 
    }
    
}