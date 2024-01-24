package com.stuypulse.robot.subsystems.amper;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AmperImpl extends Amper {
    
    private final CANSparkMax scoreMotor;
    private final CANSparkMax liftMotor;
    private final RelativeEncoder liftEncoder;

    private final DigitalInput alignedSwitch;
    private final DigitalInput minSwitch;
    private final DigitalInput maxSwitch;
    private final DigitalInput ampIRSensor;

    public AmperImpl() {
        scoreMotor = new CANSparkMax(Ports.Amper.SCORE, MotorType.kBrushless);
        liftMotor = new CANSparkMax(Ports.Amper.LIFT, MotorType.kBrushless);
        liftEncoder = liftMotor.getEncoder();

        alignedSwitch = new DigitalInput(Ports.Amper.ALIGNED_SWITCH_CHANNEL);
        minSwitch = new DigitalInput(Ports.Amper.MIN_LIFT_CHANNEL);
        maxSwitch = new DigitalInput(Ports.Amper.MAX_LIFT_CHANNEL);
        ampIRSensor = new DigitalInput(Ports.Amper.AMP_IR_CHANNEL);

        Motors.Amper.LIFT_MOTOR.configure(liftMotor);
        Motors.Amper.SCORE_MOTOR.configure(scoreMotor);
        liftEncoder.setPositionConversionFactor(Settings.Amper.Lift.ENCODER_CONVERSION);
    }

    @Override
    public boolean hasNote() {
        return !ampIRSensor.get();
    }

    @Override
    public boolean liftAtBottom() {
        return !minSwitch.get();
    }

    @Override
    public boolean liftAtTop() {
        return !maxSwitch.get();
    }

    @Override
    public double getLiftHeight() {
        return liftEncoder.getPosition();
    }

    @Override
    public boolean touchingAmp() {
        return !alignedSwitch.get();
    }

    @Override
    public void score() {
        scoreMotor.set(Settings.Amper.Score.ROLLER_SPEED);
    }

    @Override
    public void intake() {
        scoreMotor.set(-Settings.Amper.Score.ROLLER_SPEED);
    }

    @Override
    public void stopLift() {
        liftMotor.stopMotor();
    }

    @Override
    public void stopRoller() {
        scoreMotor.stopMotor();
    }

    @Override
    public void setLiftVoltageImpl() {
        liftMotor.setVoltage(liftController.update(targetHeight.get(), liftEncoder.getPosition()));
    }
    
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Amper/Intake Speed", scoreMotor.get());
        SmartDashboard.putNumber("Amper/Lift Speed", liftMotor.get());
        SmartDashboard.putNumber("Amper/Intake Current", scoreMotor.getOutputCurrent());
        SmartDashboard.putNumber("Amper/Lift Current", liftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Amper/Lift Height", getLiftHeight());

        if (liftAtBottom() || liftAtTop()) {
            stopLift();
        }
    }

}
