package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings.Amper.Score;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmperScoreAmp extends SequentialCommandGroup {
    public AmperScoreAmp() {
        addCommands(
            new AmperToHeight(Score.AMP_SCORE_HEIGHT.get()),
            new AmperOuttake()
        );
    }

}
