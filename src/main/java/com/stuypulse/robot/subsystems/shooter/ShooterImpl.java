/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.HighPassFilter;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Motors.StatusFrame;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Shooter.Feedforward;
import com.stuypulse.robot.constants.Settings.Shooter.PID;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.StupidFilter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

public class ShooterImpl extends Shooter {

    private final CANSparkFlex topMotor;
    private final CANSparkFlex bottomMotor;

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private final Controller topController;
    private final Controller bottomController;
    
    private final IStream rpmChange;
    
    private final StupidFilter topVel;
    private final StupidFilter bottomVel;

    protected ShooterImpl() {
        topMotor = new CANSparkFlex(Ports.Shooter.TOP_MOTOR, MotorType.kBrushless);
        bottomMotor = new CANSparkFlex(Ports.Shooter.BOTTOM_MOTOR, MotorType.kBrushless);

        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();

        topEncoder.setVelocityConversionFactor(1.0);
        bottomEncoder.setVelocityConversionFactor(1.0);

        topController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
        bottomController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
        
        rpmChange = IStream.create(this::getAverageShooterRPM)
            .filtered(new HighPassFilter(Settings.Shooter.RPM_CHANGE_RC));

        topVel = new StupidFilter("Top Shooter Velocity");
        bottomVel = new StupidFilter("Bottom Shooter Velocity");
        
        Motors.disableStatusFrames(topMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_POSIITION, StatusFrame.ABS_ENCODER_VELOCITY);
        Motors.disableStatusFrames(bottomMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_POSIITION, StatusFrame.ABS_ENCODER_VELOCITY);

        Motors.Shooter.TOP_SHOOTER.configure(topMotor);
        Motors.Shooter.BOTTOM_SHOOTER.configure(bottomMotor);
    }

    @Override
    public double getTopShooterRPM() {
        return topVel.get(topEncoder.getVelocity());
    }

      @Override
    public double getBottomShooterRPM() {
        return bottomVel.get(bottomEncoder.getVelocity());
    }

    @Override
    public boolean noteShot() {
        return getAverageShooterRPM() > 0 && rpmChange.get() < -Settings.Shooter.RPM_CHANGE_DIP_THRESHOLD;
    }

    @Override
    public void periodic() {
        super.periodic();

        topController.update(getTopTargetRPM(), getTopShooterRPM());
        bottomController.update(getBottomTargetRPM(), getBottomShooterRPM());

        if (getTopTargetRPM() == 0 && getBottomTargetRPM() == 0) {
            topMotor.stopMotor();
            bottomMotor.stopMotor();
        } else {
            topMotor.setVoltage(topController.getOutput());
            bottomMotor.setVoltage(bottomController.getOutput());
        }

        SmartDashboard.putNumber("Shooter/Bottom RPM", getBottomShooterRPM());
        SmartDashboard.putNumber("Shooter/Top RPM", getTopShooterRPM());

        SmartDashboard.putNumber("Shooter/Bottom Target RPM", getBottomTargetRPM());
        SmartDashboard.putNumber("Shooter/Top Target RPM", getTopTargetRPM());
        
        SmartDashboard.putNumber("Shooter/Bottom Error", bottomController.getError());
        SmartDashboard.putNumber("Shooter/Top Error", topController.getError());

        SmartDashboard.putNumber("Shooter/Left Voltage", topMotor.getBusVoltage() * topMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Right Voltage", bottomMotor.getBusVoltage() * bottomMotor.getAppliedOutput());

        SmartDashboard.putNumber("Shooter/Top Current", topMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Bottom Current", bottomMotor.getOutputCurrent());

        SmartDashboard.putNumber("Shooter/RPM Change", rpmChange.get());
        SmartDashboard.putBoolean("Shooter/Note Shot", noteShot());

        SmartDashboard.putNumber("Shooter/Distance", Odometry.getInstance().getPose().getTranslation().minus(Field.getAllianceSpeakerPose().getTranslation()).getNorm());
    }
}
