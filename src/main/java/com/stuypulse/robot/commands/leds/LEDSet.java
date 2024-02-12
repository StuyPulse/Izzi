package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;

import edu.wpi.first.wpilibj2.command.Command;

public class LEDSet extends Command {

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private final LEDInstruction ledInstruction;
    private final LEDController ledController;

    public LEDSet(LEDInstruction ledInstruction) {
        this.ledInstruction = ledInstruction;
        this.ledController = LEDController.getInstance();

        addRequirements(ledController);
    }
    
    @Override
    public void execute() {
        ledController.runLEDInstruction(ledInstruction);
    }
}
