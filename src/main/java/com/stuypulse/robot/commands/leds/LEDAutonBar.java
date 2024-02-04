package com.stuypulse.robot.commands.leds;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDAutonChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDAutonBar extends Command {

    @Override
    public boolean runsWhenDisabled() {
        return Settings.LED.LED_AUTON_CHOOSE_TOGGLE.get();
    }

    private LEDController controller;

    public LEDAutonBar() {
        this.controller = LEDController.getInstance();
    }

    @Override
    public void initialize() {
        controller.forceSetLED(new LEDAutonChooser(new PathPlannerAuto(RobotContainer.getAutonomousCommandNameStatic())));
    }
}
