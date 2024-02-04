package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;

import edu.wpi.first.wpilibj2.command.Command;

// Note: This command should be used with whileTrue as opposed to setLED being only onTrue/False
// should be used with LEDInstructions like LEDSetPulse, RichieMode, LEDRainbow, basically any moving LEDs
public class LEDSetContinous extends Command {
    
    private final LEDInstruction ledInstruction;
    private final LEDController ledController;

    public LEDSetContinous(LEDInstruction ledInstruction) {
        this.ledInstruction = ledInstruction;
        this.ledController = LEDController.getInstance();
        addRequirements(ledController);
    }
    
    @Override
    public void execute() {
        LEDController.getInstance().forceSetLED(ledInstruction);
    }
}
