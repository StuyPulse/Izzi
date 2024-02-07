# LED Code Breakdown

## [SLColor.java](src/main/java/com/stuypulse/robot/util/SLColor.java)
**Utility** wrapper class for colors that handles various Color classes used in FRC (java.awt.Color, wpilibj.util.Color, wpilibj.util.Color8Bit). Stores them in rbg values from 0-255. 

```java
SLColor red = new SLColor(255, 0, 0); // Creates red color
SLColor red = new SLColor(wplibj.util.Color.RED); // Creates red color from wpilibj.util.Color
SLColor red = new SLColor(java.awt.Color.RED); // Creates red color from java.awt.Color
```

## [LEDInstruction.java](src/main/java/com/stuypulse/robot/subsystems/led/LEDInstruction.java)
**Interface** class for setting LED colors; can do from very simple displays to very elaborate controlling of the LEDs.
- setLED(AddressableLEDBuffer buffer): sets the LED buffer

### Implementations:
- [LEDPulseColor.java](src/main/java/com/stuypulse/robot/subsystems/leds/instructions/LEDPulseColor.java) - Pulses between two colors. Defaults to one color and black if color2 is not set.
- [LEDRainbow.java](src/main/java/com/stuypulse/robot/subsystems/leds/instructions/LEDRainbow.java) - Cycles through the rainbow.
- [LEDSection.java](src/main/java/com/stuypulse/robot/subsystems/leds/instructions/LEDSection.java) - Creates partitions of LED sections to colors.
- [LEDSingleColor.java](src/main/java/com/stuypulse/robot/subsystems/leds/instructions/LEDSingleColor.java) - Sets single solid color.
- [LEDAlign.java](src/main/java/com/stuypulse/robot/commands/led/LEDAlign.java) - aligns LEDs based on odometry and selected auton's starting pose. (also a command)
- [RichieMode.java](src/main/java/com/stuypulse/robot/subsystems/leds/instructions/RichieMode.java) - Single pixel running animation.

## [LEDColor.java](src/main/java/com/stuypulse/robot/constants/LEDColor.java)
Constants to store predefined LED instructions so that they can be used in commands. 

```java
//inside constants/LEDColor.java
LEDInstruction RED = new LEDSingleColor(SLColor.RED);
LEDInstruction RAINBOW = new LEDRainbow();
LEDInstruction PULSE = new LEDPulseColor(SLColor.RED, SLColor.BLUE);
```

## [LEDController.java](src/main/java/com/stuypulse/robot/led/LEDController.java)
**Subsystem** that contains AddressableLED (physical LED strip hardware) and AddressableLEDBuffer (array structure to store RGB values for each LED) to control LEDs.

Methods:
- `getInstance()` - returns the singleton instance of the LEDController.
- `forceSet(LEDInstruction instruction)` - sets the LEDs to the given instruction.
- `getDefaultColor()` - returns the default color of the LEDs based on the current mode (teleop, auto, etc.).
- `periodic()` - updates the LEDs based on the current mode (teleop, auto, etc.) or overrides it with a forced instruction.

```java
//usage in other files
LEDController ledController = new LEDController();
ledController.forceSet(LEDColor.RED); // sets LEDs to red
```

## [LEDSet.java](src\main\java\com\stuypulse\robot\commands\leds\LEDSet.java)  
**Command** to set LEDs to an LEDInstruction.

```java
//example usage in RobotContainer.java
driver.getLeftButton().whenPressed(new LEDSet(LEDColor.RED));
```