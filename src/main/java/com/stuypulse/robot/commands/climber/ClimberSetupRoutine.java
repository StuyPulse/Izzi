package com.stuypulse.robot.commands.climber;
import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.commands.swerve.SwerveDriveCenter;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimberSetupRoutine extends SequentialCommandGroup {

    public ClimberSetupRoutine() {
        addCommands(
            new AmperToHeight(Settings.Amper.Lift.MIN_HEIGHT),
            new ClimberToTop(),
            new SwerveDriveCenter(),
            new ClimberToBottom()
        );
    }
}