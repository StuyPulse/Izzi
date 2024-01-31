package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.constants.LEDColor;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
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
 * 
 * 
 * 
 * 
 * 
 * 
LED Code Breakdown:

subsystem:
LEDController.java 
-  AddressableLED
-  AddressableLEDBuffer 
 - forceSetLED(LEDInstruction instruction) : sets color of LEDs for short time
 - getDefaultColor() : determines LED color if it is not set forcefully
 - periodic() : sets LED color to default color if in teleop

LEDInstruction.java (advanced ways you can change the colors on the LED)
| (children)
- pulsing
- streaming
- singleColor
- rainbow
- sections

util:
SLColor.java (color class that we use to set the colors)
- rgb (0-255)
- SLColor constants to be used 

constants:
LEDColor.java (place where you store your LEDInstructions)

commands:
LEDSet.java (the command you use to set LEDS)
LEDAlign.java (odometry, streaming, tbd)
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

    public LEDController() {
        leds = new AddressableLED(Ports.LEDController.PORT);
        ledsBuffer = new AddressableLEDBuffer(Settings.LED.LED_LENGTH); 

        leds.setLength(ledsBuffer.getLength());
        leds.setData(ledsBuffer);
        leds.start();

        SmartDashboard.putData(instance);
    }


    public void forceSetLED(LEDInstruction instruction) {
        instruction.setLED(ledsBuffer);
        leds.setData(ledsBuffer);
    }

    public LEDInstruction getDefaultColor() {
        return LEDColor.RED;
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            forceSetLED(getDefaultColor());
        }
    }
}
