package com.stuypulse.robot.commands.climber;
import com.stuypulse.robot.commands.amper.AmperScore;
import com.stuypulse.robot.commands.amper.AmperSetLiftConstraints;
import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimberScoreRoutine extends SequentialCommandGroup {
    public ClimberScoreRoutine() {
        addCommands(
            // lower climber and raise lift (after delay) simultaneously
            ClimberToHeight.untilDone(Settings.Climber.MIN_HEIGHT)
                .alongWith(
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        // slow down lift
                        new AmperSetLiftConstraints(1.0, 2.0),
                        AmperToHeight.untilDone(Score.TRAP_SCORE_HEIGHT.get()),
                        new AmperSetLiftConstraints()
                    )),
            AmperScore.forSeconds(2)
        );
    }
}