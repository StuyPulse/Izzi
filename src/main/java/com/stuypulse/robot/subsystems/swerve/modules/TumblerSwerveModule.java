/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.swerve.modules;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.util.FilteredRelativeEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TumblerSwerveModule extends SwerveModule {

    public static SwerveModule[] getModules() {
        return new SwerveModule[] {
            new TumblerSwerveModule("Front Right", Swerve.FrontRight.MODULE_OFFSET, Rotation2d.fromDegrees(-153.632812 + 180), 15, 14, 4),
            new TumblerSwerveModule("Front Left",  Swerve.FrontLeft.MODULE_OFFSET,  Rotation2d.fromDegrees(147.919922 + 180),  17, 16, 2),
            new TumblerSwerveModule("Back Left",   Swerve.BackLeft.MODULE_OFFSET,   Rotation2d.fromDegrees(73.125 + 180),   11, 10, 3),
            new TumblerSwerveModule("Back Right",  Swerve.BackRight.MODULE_OFFSET,  Rotation2d.fromDegrees(-2.02184 + 180),  13, 12, 1)
        };
    }

    // data
    private final Rotation2d angleOffset;

    // turn
    private final CANSparkMax turnMotor; 
    private final CANcoder turnEncoder;

    // drive
    private final CANSparkFlex driveMotor;
    private final RelativeEncoder driveEncoder; 
    
    // controllers
    private final Controller driveController; 
    private final AngleController turnController;
   
    private TumblerSwerveModule(String id, Translation2d translationOffset, Rotation2d angleOffset, int turnID, int driveID, int encoderID) {
        super(id, translationOffset);

        this.angleOffset = angleOffset;
        
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveMotor = new CANSparkFlex(driveID, MotorType.kBrushless);

        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);

        turnEncoder = new CANcoder(encoderID);
        driveEncoder = new FilteredRelativeEncoder(driveMotor);

        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        
        driveController = new PIDController(0.002794, 0.0, 0.0)
            .add(new MotorFeedforward(0.17313, 1.8573, 0.29554).velocity());

        turnController = new AnglePIDController(6.0, 0.0, 0.15)
            .setOutputFilter(x -> -x);

        Motors.Swerve.DRIVE_CONFIG.configure(driveMotor);
        Motors.Swerve.TURN_CONFIG.configure(turnMotor);
    }

    @Override
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble()).minus(angleOffset);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    @Override
    public void addInstrumentToOrchestra(Orchestra o) {}

    @Override
    public void periodic() {
        super.periodic();

        turnController.update(
            Angle.fromRotation2d(getTargetState().angle),
            Angle.fromRotation2d(getAngle()));
        
        driveController.update(
            getTargetState().speedMetersPerSecond,
            getVelocity());

        if (Math.abs(driveController.getSetpoint()) < Settings.Swerve.MODULE_VELOCITY_DEADBAND) {
            driveMotor.setVoltage(0);
            turnMotor.setVoltage(0);
        } else {
            driveMotor.setVoltage(driveController.getOutput());
            turnMotor.setVoltage(turnController.getOutput());
        }

        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Speed", getVelocity());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Raw Encoder Angle", Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()));
    }
}

