package com.stuypulse.robot.commands.conveyor;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ConveyorScoreNote extends Command {
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Amper amper;

    public ConveyorScoreNote() {
        shooter = Shooter.getInstance();
        conveyor = Conveyor.getInstance();
        amper = Amper.getInstance();
        addRequirements(shooter, conveyor, amper);
    };

    public void execute(){
        if (amper.hasNote() && amper.touchingAmp()){
            amper.score();
        }
        else if (conveyor.isNoteAtShooter()) {
            shooter.setLeftTargetRPM(Settings.Shooter.PODIUM_SHOT_LEFT_RPM);
            shooter.setRightTargetRPM(Settings.Shooter.PODIUM_SHOT_RIGHT_RPM);
        }
    }
}