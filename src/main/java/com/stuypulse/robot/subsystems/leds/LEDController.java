package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*-
 * Contains:
 *      - forceSetLED() : sets color of LEDs for short time
 *      - getDefaultColor() : determines LED color if it is not set
 *      - periodic() : sets LED color to default color if in teleop
 */

public class LEDController extends SubsystemBase {
    private static LEDController instance; 

    static {
        instance = new LEDController();
    }
    
    public static LEDController getInstance() {
        return instance;
    }
    
    private AddressableLED leds;
    private AddressableLEDBuffer ledsBuffer;
    private LEDInstruction currentInstruction;

    protected LEDController() {
        leds = new AddressableLED(Ports.LEDController.PORT);
        ledsBuffer = new AddressableLEDBuffer(Settings.LED.LED_LENGTH); 

        leds.setLength(ledsBuffer.getLength());
        leds.setData(ledsBuffer);
        leds.start();

        currentInstruction = LEDInstructions.OFF;

        SmartDashboard.putData(instance);
    }


    public void runLEDInstruction(LEDInstruction instruction) {
        currentInstruction = instruction;
        instruction.setLED(ledsBuffer);
        leds.setData(ledsBuffer);
    }

    public LEDInstruction getDefaultColor() {
        switch (Manager.getInstance().getScoreLocation()) {
            case SPEAKER: return LEDInstructions.YELLOW;
            case AMP: return LEDInstructions.ORANGE;
            case TRAP: return LEDInstructions.PINK;
            default: return LEDInstructions.OFF;
        }
    }

    @Override
    public void periodic() {
        if (currentInstruction == getDefaultColor() || Robot.getMatchState() != MatchState.DISABLE) { 
            runLEDInstruction(getDefaultColor());
        }
        else {
            runLEDInstruction(currentInstruction);     
        }  
        SmartDashboard.putString("LED/ Current Instruction", currentInstruction.toString());
    }
}
