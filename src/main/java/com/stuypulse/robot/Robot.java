/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.leds.instructions.LEDAlign;
import com.stuypulse.robot.subsystems.leds.instructions.LEDAutonChooser;

import edu.wpi.first.wpilibj.DataLogManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer robot;
    private CommandScheduler scheduler;
    private Command auto;

    /************************/
    /*** ROBOT SCHEDULING ***/
    /************************/

    @Override
    public void robotInit() {
        DataLogManager.start();

        scheduler = CommandScheduler.getInstance();

        robot = new RobotContainer();

        SmartDashboard.putString("Robot State", "DISABLED");
    }

    @Override
    public void robotPeriodic() {
        scheduler.run();
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Robot State", "DISABLED");
    }

    @Override
    public void disabledPeriodic() {
        if (robot.intake.hasNote()) {
            DriverStation.reportWarning("Intake IR sensor reporting note while disabled!", false);
        }

        if (robot.amper.hasNote()) {
            DriverStation.reportWarning("Amper IR sensor reporting note while disabled!", false);
        }

        if (robot.conveyor.isNoteAtShooter()) {
            DriverStation.reportWarning("Shooter IR sensor reporting note while disabled!", false);
        }

        if (Settings.LED.LED_AUTON_TOGGLE.get()) {
            scheduler.schedule(new LEDSet(new LEDAlign()));
        }
        else {
            scheduler.schedule(new LEDSet(new LEDAutonChooser()));
        }
    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }

        SmartDashboard.putString("Robot State", "AUTON");
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        if (auto != null) {
            auto.cancel();
        }

        SmartDashboard.putString("Robot State", "TELEOP");
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        SmartDashboard.putString("Robot State", "TEST");
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
