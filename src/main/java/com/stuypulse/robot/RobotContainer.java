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
import com.stuypulse.robot.commands.auton.ADE.*;
import com.stuypulse.robot.commands.auton.CBADE.*;
import com.stuypulse.robot.commands.auton.CHFG.FivePieceCHGF;
import com.stuypulse.robot.commands.auton.DE.*;
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
import com.stuypulse.robot.constants.Settings.Amper.Score;
import com.stuypulse.robot.constants.Settings.Driver;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.climber.*;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

    public final PowerDistribution pdp = new PowerDistribution();

    // Autons
    private static SendableChooser<Command> autonChooser;

    // RobotContainer
    public RobotContainer() {
        autonChooser = new SendableChooser<Command>();

        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        LiveWindow.disableAllTelemetry();

        if (Robot.isReal()) CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 80, 60, 30);

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
                    .deadlineWith(new LEDSet(LEDInstructions.INTAKE))
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
            .whileTrue(new SwerveDriveToShoot()
                    .deadlineWith(new LEDSet(LEDInstructions.SPEAKER_ALIGN))
                .andThen(new ShooterWaitForTarget()
                    .withTimeout(1.5))
                .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop());

        // note to amper and align then score
        driver.getLeftBumper()
            .whileTrue(new AmpScoreRoutine())
            .onFalse(new AmperStop());

        // score speaker no align
        driver.getRightMenuButton()
            .onTrue(new ShooterPodiumShot())
            .onTrue(new ShooterWaitForTarget()
                    .withTimeout(1.5)
                .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop());

        // score amp no align
        driver.getLeftMenuButton()
            .whileTrue(ConveyorToAmp.withCheckLift()
                .andThen(AmperToHeight.untilDone(Lift.AMP_SCORE_HEIGHT))
                .andThen(new AmperScore()
                    .deadlineWith(new LEDSet(LEDInstructions.AMP_SCORE))))
            .onFalse(new AmperStop())
            .onFalse(new AmperToHeight(Lift.MIN_HEIGHT));

        // score trap
        driver.getLeftButton()
            .onTrue(new AmperScoreTrap())
            .onFalse(new AmperStop());

        driver.getDPadLeft()
            .whileTrue(new FastAlignShootSpeakerRelative(-45)
                .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop());

        // lift to trap
        driver.getDPadRight()
            .onTrue(new ConditionalCommand(new ConveyorToAmp(), new DoNothingCommand(), () -> Intake.getInstance().hasNote())
                .andThen(new AmperToHeight(Lift.TRAP_SCORE_HEIGHT)));

        // reset heading
        driver.getDPadUp()
            .onTrue(new SwerveDriveResetHeading());
        
        // trap outtake
        driver.getDPadDown()
            .onTrue(new AmperScoreSpeed(-Score.TRAP_SPEED))
            .onFalse(new AmperStop());

        // automatic swerve drive
        // driver.getTopButton()
        //     // on command start
        //     .onTrue(new BuzzController(driver, Assist.BUZZ_INTENSITY)
        //         .deadlineWith(new LEDSet(LEDInstructions.AUTO_SWERVE)))
                
        //     .onTrue(new SwerveDriveAutomatic(driver)
        //         // after command end
        //         .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY)
        //             .deadlineWith(new LEDSet(LEDInstructions.AUTO_SWERVE)))

        //         .andThen(new WaitCommand(Driver.Drive.BUZZ_DURATION))

        //         .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY)
        //             .deadlineWith(new LEDSet(LEDInstructions.AUTO_SWERVE))));

        driver.getTopButton()
            .whileTrue(new SwerveDriveAutoFerry(driver));
            // .whileTrue(new SwerveDriveToShootMoving());

        // climb
        driver.getRightButton()
            .whileTrue(new SwerveDriveToClimb()
                .deadlineWith(new LEDSet(LEDInstructions.TRAP_ALIGN)));

        // drive to chain
        driver.getBottomButton()
            .whileTrue(new SwerveDriveDriveToChain()
                .deadlineWith(new LEDSet(LEDInstructions.TRAP_ALIGN)));
    }

    private void configureOperatorBindings() {
        // climber
        new Trigger(() -> operator.getLeftStick().magnitude() > Settings.Operator.DEADBAND.get())
            .whileTrue(new ClimberDrive(operator));

        // climber LED
        new Trigger(() -> operator.getLeftY() > Settings.Operator.DEADBAND.get())
            .onTrue(new LEDSet(LEDInstructions.CLIMB_UP)
                .withTimeout(1.0));

        // move note to amp
        new Trigger(() -> operator.getLeftY() > 0.25)
            .onTrue(new ConveyorToAmp()
                .andThen(new ShooterStop()));

        
        new Trigger(() -> operator.getRightStick().magnitude() > Settings.Operator.DEADBAND.get())
            .whileTrue(new AmperLiftDrive(operator));

        operator.getLeftTriggerButton()
            .onTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());
        operator.getRightTriggerButton()
            .whileTrue(new IntakeAcquire()
                .deadlineWith(new LEDSet(LEDInstructions.INTAKE))
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
            .whileTrue(new LEDSet(LEDInstructions.ATTENTION));
        
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

        AutonConfig HGF = new AutonConfig("4 HGF", FourPieceHGF::new,
        "Start to H (HGF)", "H to HShoot (HGF)", "HShoot to G (HGF)", "G to Shoot (HGF)", "GShoot to F (HGF)", "F to Shoot (HGF)");
        
        AutonConfig TrackingCBAE = new AutonConfig("Tracking 5 CBAE Podium", FivePieceTrackingCBAE::new,
            "Preload to C", "C to B", "B to A", "A to E", "E to Shoot");   

        AutonConfig CBAED = new AutonConfig("5 CBAE", SixPieceCBAED::new,
        "Preload to C Close", "Close Preload to C", "C to B", "B to A","A to E", "E to Shoot", "Shoot to D (CBAED)", "D to Shoot");

        AutonConfig CBAED_OLD = new AutonConfig("5 CBAE Old", SixPieceCBAEDOld::new,
        "Preload to C", "C to B", "B to A", "A to E", "E to Shoot", "Shoot to D (CBAED)", "D to Shoot");

        AutonConfig CHGF = new AutonConfig("4.5 Piece CHGF", FivePieceCHGF::new,
        "Preload to C", "CShoot To H (CHGF)", "H to HShoot (HGF)", "HShoot to G (HGF)", "G to Shoot (HGF)", "GShoot to F (HGF)");

        // AutonConfig ADEF = new AutonConfig("4.5 Piece ADEF", FourPieceADEF::new, 
        // "Preload to A", "A to D", "D to Shoot", "Shoot to E", "E to Shoot", "Shoot To F (ADEF)", "F To Shoot (ADEF)");

        // AutonConfig ADE = new AutonConfig("3 ADE", ThreePieceADE::new,
        //     "Preload to A", "A to D", "D to Ferry Shot", "Ferry Shot to E", "E to Shoot");
        
        // AutonConfig DE = new AutonConfig("2 DE", TwoPieceDE::new,
        //     "Preload Shot to D", "D to Ferry Shot", "Ferry Shot to E", "E to Shoot");

        // AutonConfig PodiumCloseCBAE = new AutonConfig("Podium Close 5 Piece CBAE", FivePiecePodiumForwardCBAE::new, 
        // "Forward First Piece to C", "C to B 2", "B to A","A to E", "E to Shoot");
        
        HGF.registerDefaultBlue(autonChooser)
            .registerRed(autonChooser);

        CBAED
            .registerBlue(autonChooser)
            .registerRed(autonChooser);

        CHGF
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
