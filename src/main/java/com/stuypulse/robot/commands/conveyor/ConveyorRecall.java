package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;
import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;

public class ConveyorRecall extends Command {

    private final Amper amper;
    private final Intake intake;
    private final Shooter shooter;
    private final Conveyor conveyor;

    private final BStream noteAtConveyor;

    private boolean noteAtShooter;
    private boolean noteAtAmp;

    public ConveyorRecall() {
        amper = Amper.getInstance();
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();
        conveyor = Conveyor.getInstance();

        noteAtConveyor = BStream.create(() -> intake.hasNote())
            .filtered(new BDebounce.Rising(Settings.Conveyor.RECALL_DEBOUNCE)); 
        
        addRequirements(amper, intake, shooter, conveyor);
    }
    
    @Override
    public void initialize() {
        noteAtShooter = conveyor.isNoteAtShooter(); 
        noteAtAmp = amper.hasNote();
    }

    @Override
    public void execute() {
        intake.deacquire();

        if (noteAtShooter) {
            conveyor.toAmp();
        }
        
        if (noteAtAmp) {
            shooter.setLeftTargetRPM(-Settings.Shooter.BACKWARDS_LEFT_RPM.get());
            shooter.setRightTargetRPM(-Settings.Shooter.BACKWARDS_RIGHT_RPM.get());

            conveyor.toShooter();
            amper.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
        amper.stopRoller();

        shooter.setLeftTargetRPM(Settings.Shooter.PODIUM_SHOT_LEFT_RPM);
        shooter.setRightTargetRPM(Settings.Shooter.PODIUM_SHOT_RIGHT_RPM);
    }
    
    @Override
    public boolean isFinished() {
        return noteAtConveyor.get() || (!noteAtShooter && !noteAtAmp);
    }
}