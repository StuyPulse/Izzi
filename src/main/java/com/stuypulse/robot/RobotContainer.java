/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.commands.amper.*;
import com.stuypulse.robot.commands.auton.*;
import com.stuypulse.robot.commands.auton.CBADE.*;
import com.stuypulse.robot.commands.auton.GHF.*;
import com.stuypulse.robot.commands.auton.HGF.*;
import com.stuypulse.robot.commands.auton.tests.*;
import com.stuypulse.robot.commands.climber.*;
import com.stuypulse.robot.commands.conveyor.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.*;
import com.stuypulse.robot.commands.notealignment.*;
import com.stuypulse.robot.commands.shooter.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.constants.Settings.Driver;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.climber.*;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDPulseColor;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.util.SLColor;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    public final AprilTagVision vision = AprilTagVision.getInstance();
    public final NoteVision noteVision = NoteVision.getInstance();
    public final Odometry odometry = Odometry.getInstance();

    public final Climber climber = Climber.getInstance();
    public final Amper amper = Amper.getInstance();
    public final LEDController leds = LEDController.getInstance();
    public final Conveyor conveyor = Conveyor.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser;

    // RobotContainer
    public RobotContainer() {
        autonChooser = new SendableChooser<Command>();

        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        SmartDashboard.putData("Gamepads/Driver", driver);
        SmartDashboard.putData("Gamepads/Operator", operator);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        leds.setDefaultCommand(new LEDDefaultMode());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureOperatorBindings();
        configureDriverBindings();
    }

    private void configureDriverBindings() {
        // intaking with swerve pointing at note
        // driver.getRightTriggerButton()
        //     .whileTrue(new WaitCommand(Settings.Intake.TELEOP_DRIVE_STARTUP_DELAY)
        //         .andThen(new IntakeAcquire()
        //             .deadlineWith(new LEDSet(LEDInstructions.DARK_BLUE)))
        //         .andThen(new BuzzController(driver)
        //             .alongWith(new LEDSet(LEDInstructions.PICKUP)
        //                 .withTimeout(3.0))))
        //     .whileTrue(new SwerveDriveToNote());
        
        // intaking
        driver.getRightTriggerButton()
            .whileTrue(new IntakeAcquire()
                    .deadlineWith(new LEDSet(LEDInstructions.DARK_BLUE))
                .andThen(new BuzzController(driver)
                    .alongWith(new LEDSet(LEDInstructions.PICKUP)
                        .withTimeout(3.0))));
        
        driver.getLeftTriggerButton()
            .onTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());

        // note to shooter and align
        // then shoot
        driver.getRightBumper()
            .onTrue(new ShooterPodiumShot())
            .whileTrue(new WaitCommand(Settings.Shooter.TELEOP_SHOOTER_STARTUP_DELAY)
                .andThen(new SwerveDriveToShoot()
                    .deadlineWith(new LEDSet(LEDInstructions.ASSIST_FLASH)))
                .andThen(new ShooterWaitForTarget()
                    .withTimeout(1.5))
                .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop())
            .onFalse(new ShooterStop());

        // note to amper and align then score
        driver.getLeftBumper()
            .whileTrue(new AmpScoreRoutine())
            .onFalse(new AmperToHeight(Settings.Amper.Lift.MIN_HEIGHT))
            .onFalse(new AmperStop());

        // score speaker no align
        driver.getRightMenuButton()
            .onTrue(new ShooterPodiumShot())
            .onTrue(new ShooterWaitForTarget()
                    .withTimeout(1.5)
                .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop())
            .onFalse(new ShooterStop());
            
        // score amp no align
        driver.getLeftMenuButton()
            .whileTrue(ConveyorToAmp.withCheckLift()
                .andThen(AmperToHeight.untilDone(Lift.AMP_SCORE_HEIGHT))
                .andThen(new AmperScore()))
            .onFalse(new AmperStop())
            .onFalse(new AmperToHeight(Lift.MIN_HEIGHT));

        // score trap
        driver.getLeftButton()
            .onTrue(new AmperScoreTrap())
            .onFalse(new AmperStop());

        driver.getDPadRight()
            .whileTrue(new TrapScoreRoutine())
            // .onFalse(new AmperToHeight(Lift.MIN_HEIGHT))
            .onFalse(new AmperStop());

        driver.getDPadUp()
            .onTrue(new SwerveDriveResetHeading());

        driver.getTopButton()
            // on command start
            .onTrue(new BuzzController(driver, Assist.BUZZ_INTENSITY)
                .deadlineWith(new LEDSet(LEDInstructions.GREEN)))
                
            .onTrue(new SwerveDriveAutomatic(driver)
                // after command end
                .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY)
                    .deadlineWith(new LEDSet(LEDInstructions.GREEN)))

                .andThen(new WaitCommand(Driver.Drive.BUZZ_DURATION))
                
                .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY)
                    .deadlineWith(new LEDSet(LEDInstructions.GREEN))));

        driver.getRightButton()
            .onTrue(new AmperToHeight(Lift.MIN_HEIGHT))
            .whileTrue(SwerveDriveToPose.toClimb());

        driver.getBottomButton()
            .whileTrue(new SwerveDriveDriveToChain());
    }

    private void configureOperatorBindings() {
        new Trigger(() -> operator.getLeftStick().magnitude() > Settings.Operator.DEADBAND.get())
                .whileTrue(new ClimberDrive(operator));

        new Trigger(() -> operator.getRightStick().magnitude() > Settings.Operator.DEADBAND.get())
                .whileTrue(new AmperLiftDrive(operator));

        operator.getLeftTriggerButton()
            .onTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());
        operator.getRightTriggerButton()
            .whileTrue(new IntakeAcquire()
                .deadlineWith(new LEDSet(LEDInstructions.DARK_BLUE))
                .andThen(new BuzzController(driver)
                    .alongWith(new LEDSet(LEDInstructions.PICKUP)
                        .withTimeout(3.0))));

        operator.getLeftBumper()
            .onTrue(new ConveyorToAmp())
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop())
            .onFalse(new AmperStop());

        operator.getTopButton()
            .onTrue(new AmperScore())
            .onTrue(new ConveyorShoot())
            .onFalse(new AmperStop())
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop());

        operator.getDPadUp()
            .whileTrue(new AmperLiftFineAdjust(operator));
        operator.getDPadDown()
            .whileTrue(new AmperLiftFineAdjust(operator));

        operator.getDPadRight()
            .onTrue(new GandalfToShoot())
            .onFalse(new ConveyorStop());
        operator.getDPadLeft()
            .onTrue(new GandalfToAmp())
            .onFalse(new ConveyorStop());
        

        operator.getRightButton()
                .onTrue(new AmperToHeight(Settings.Amper.Lift.AMP_SCORE_HEIGHT));
        operator.getLeftButton()
                .onTrue(new AmperToHeight(Settings.Amper.Lift.TRAP_SCORE_HEIGHT));
        operator.getBottomButton()
                .onTrue(new AmperToHeight(Settings.Amper.Lift.MIN_HEIGHT));

        // human player attention button
        operator.getRightMenuButton()
            .whileTrue(new LEDSet(LEDInstructions.PULSE_PURPLE));
        
        operator.getLeftMenuButton()
            .onTrue(new AmperToConveyor())
            .onFalse(new AmperStop());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());

        autonChooser.addOption("Mobility", new Mobility());

        AutonConfig CBAE = new AutonConfig("5 Piece CBAE", FivePieceCBAE::new,
        "First Piece To C", "C to B", "B To A", "A To E", "E To Shoot");
        
        AutonConfig BLAY_CBAE = new AutonConfig("Blay 5 Piece CBAE", FivePieceCBAE::new,
        "Blay First Piece To C", "C to B", "B To A", "A To E", "E To Shoot");

        AutonConfig HGF = new AutonConfig("4 Piece HGF", FourPieceHGF::new,
        "Start To H (HGF)", "H To HShoot (HGF)", "HShoot To G (HGF)", "G To Shoot (HGF)", "GShoot To F (HGF)", "F To Shoot (HGF)");
        
        AutonConfig TrackingCBAE = new AutonConfig("Tracking 5 Piece CBAE", FivePieceTrackingCBAE::new,
            "First Piece To C", "C to B", "B To A", "A To E", "E To Shoot");   

        AutonConfig SubwooferCBAE = new AutonConfig("Subwoofer 5 Piece CBAE", FivePiecePodiumForwardCBAE::new, 
        "Forward First Piece to C", "C to B 2", "B To A","A To E", "E To Shoot");

        AutonConfig PodiumCBAE = new AutonConfig("Podium 5 Piece CBAE", FivePiecePodiumCBAE::new, 
        "Blay First Piece To C", "C to B", "B To A","A To E", "E To Shoot");

        AutonConfig PodiumCloseCBAE = new AutonConfig("Podium Close 5 Piece CBAE", FivePiecePodiumForwardCBAE::new, 
        "Forward First Piece to C", "C to B 2", "B To A","A To E", "E To Shoot");

        // CBAE.registerBlue(autonChooser)
        //     .registerRed(autonChooser);
        
        // BLAY_CBAE
        //     .registerBlue(autonChooser)
        //     .registerRed(autonChooser);
        
        HGF.registerDefaultBlue(autonChooser)
            .registerRed(autonChooser);

        TrackingCBAE
            .registerBlue(autonChooser)
            .registerRed(autonChooser);
        
        PodiumCBAE
            .registerBlue(autonChooser)
            .registerRed(autonChooser);
        
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static String getAutonomousCommandNameStatic() {
        if (autonChooser.getSelected() == null) {
            return "Do Nothing";
        }
        
        return autonChooser.getSelected().getName();
    }
}
