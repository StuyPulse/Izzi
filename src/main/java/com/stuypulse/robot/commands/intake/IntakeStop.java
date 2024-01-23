package com.stuypulse.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.stuypulse.robot.subsystems.intake.Intake;

public class IntakeStop extends InstantCommand {
    
    private Intake intake;
    
    public IntakeStop() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stop();
    }
}
