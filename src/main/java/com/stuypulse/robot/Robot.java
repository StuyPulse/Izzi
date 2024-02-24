/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.leds.LEDReset;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.robot.subsystems.leds.instructions.LEDAlign;
import com.stuypulse.robot.subsystems.leds.instructions.LEDAutonChooser;
import com.stuypulse.robot.subsystems.leds.instructions.LEDRainbow;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.revrobotics.CANSparkBase.IdleMode;

public class Robot extends TimedRobot {
    
    public static final RobotType ROBOT;

    static {
        if (Robot.isSimulation())
            ROBOT = RobotType.SIM;
        else
            ROBOT = RobotType.fromString(System.getenv("serialnum"));
    }

    private RobotContainer robot;
    private CommandScheduler scheduler;
    private Command auto;

    /************************/
    /*** ROBOT SCHEDULING ***/
    /************************/

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStar());

        DataLogManager.start();

        scheduler = CommandScheduler.getInstance();

        robot = new RobotContainer();

        SmartDashboard.putString("Robot State", "DISABLED");
        SmartDashboard.putString("Robot", ROBOT.name());
    }

    @Override
    public void robotPeriodic() {
        scheduler.run();
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        robot.intake.setIdleMode(IdleMode.kCoast);
        robot.conveyor.setIdleMode(IdleMode.kCoast);

        scheduler.schedule(new LEDSet(new LEDRainbow()));

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

        // if (Settings.LED.LED_AUTON_TOGGLE.get()) {
        //     scheduler.schedule(new LEDSet(new LEDAlign()));
        // } else {
        //     scheduler.schedule(new LEDSet(new LEDAutonChooser()));
        // }

        // reload whitelist in case of alliance change
        scheduler.schedule(new VisionReloadWhiteList());
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

        scheduler.schedule(new LEDReset());

        robot.intake.setIdleMode(IdleMode.kBrake);
        robot.conveyor.setIdleMode(IdleMode.kBrake);

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

        robot.climber.stop();
        scheduler.schedule(new LEDReset());
        scheduler.schedule(new ShooterStop());

        robot.intake.setIdleMode(IdleMode.kBrake);
        robot.conveyor.setIdleMode(IdleMode.kBrake);

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

        scheduler.schedule(new LEDReset());

        robot.intake.setIdleMode(IdleMode.kBrake);
        robot.conveyor.setIdleMode(IdleMode.kBrake);

        SmartDashboard.putString("Robot State", "TEST");
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
