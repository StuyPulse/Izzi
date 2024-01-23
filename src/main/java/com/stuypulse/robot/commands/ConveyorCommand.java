package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Conveyor.Conveyor;
import com.stuypulse.robot.subsystems.Conveyor.ConveyorLogic;

import edu.wpi.first.wpilibj2.command.Command;

public class ConveyorCommand extends Command {
    private ConveyorLogic logic;

    private Conveyor conveyor;
    private Conveyor.Direction target;

    public ConveyorCommand(ConveyorLogic logic) {
        super();

        conveyor = Conveyor.getInstance();

        // Also add requirements for intake and amp
        addRequirements(conveyor);

        this.logic = logic;
    }

    public ConveyorCommand(ConveyorLogic logic, Conveyor.Direction target) {
        super();

        conveyor = Conveyor.getInstance();
        addRequirements(conveyor);

        this.logic = logic;
        this.target = target;
    }

    public void execute() {
        if (target != null) {
            conveyor.setTarget(target);
        }

        // Get sensor values and pass into conveyor logic
        ConveyorLogic.Input in = new ConveyorLogic.Input(
            false,
            false,
            false,
            conveyor.getTarget());

        // Run conveyor logic
        ConveyorLogic.Output out = logic.run(in);

        // Apply outputs to various subsystems here
        switch (out.conveyor()) {
            case SHOOTER:
                conveyor.gandalfToShooter();
            case AMP:
                conveyor.gandalfToAmp();
            case NONE:
                conveyor.gandalfStop();
        }

        if (out.feeder()) {
            conveyor.feederForward();
        } else {
            conveyor.feederStop();
        }
    }
}
