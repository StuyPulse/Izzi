package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAcquireAndStore extends Command{
      
    private Intake intake;
    
    public IntakeAcquireAndStore() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.acquire();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote();
    }
}
