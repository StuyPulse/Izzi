package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;

public class ConveyorToAmp extends Command {
    public static Command withCheckLift() {
        return AmperToHeight.untilDone(Settings.Amper.Lift.MIN_HEIGHT)
            .andThen(new ConveyorToAmp());
    }

    private final Conveyor conveyor;
    private final Intake intake;
    private final Amper amper;

    public ConveyorToAmp() {
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();
        amper = Amper.getInstance();

        addRequirements(conveyor, intake, amper);
    }

    @Override
    public void execute() {
        conveyor.toAmp();
        intake.acquire();
        amper.score();
    }

    @Override
    public boolean isFinished() {
        return amper.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        intake.stop();
        amper.stopRoller();
    }
}


