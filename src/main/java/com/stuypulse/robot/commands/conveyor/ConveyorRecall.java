package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;
import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.subsystems.intake.Intake;

public class ConveyorRecall extends Command {

    private final Amper amper;
    private final Conveyor conveyor;
    private final Intake intake;

    private final BStream noteAtConveyor;

    private boolean noteAtShooter;
    private boolean noteAtAmp;

    public ConveyorRecall() {
        conveyor = Conveyor.getInstance();
        amper = Amper.getInstance();
        intake = Intake.getInstance();

        noteAtConveyor = BStream.create(() -> intake.hasNote())
            .filtered(new BDebounce.Rising(Settings.Conveyor.RECALL_DEBOUNCE)); 
        
        addRequirements(amper,intake);
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
            conveyor.toShooter();
            amper.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
        amper.stopRoller();
    }
    
    @Override
    public boolean isFinished() {
        return noteAtConveyor.get() || (!noteAtShooter && !noteAtAmp);
    }
}