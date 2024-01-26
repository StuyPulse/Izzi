package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings.Amper.Score;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmp extends SequentialCommandGroup {
    public ScoreAmp() {
        addCommands(
            new LiftToHeight(Score.AMP_SCORE_HEIGHT.get(), Score.AMP_SCORE_THRESHOLD.get()),
            new RunRoller(true)
        );
    }

}
