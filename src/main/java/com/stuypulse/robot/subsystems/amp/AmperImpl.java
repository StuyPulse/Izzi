package com.stuypulse.robot.subsystems.amp;
import static com.stuypulse.robot.constants.Motors.Amper.*;
import static com.stuypulse.robot.constants.Ports.Amp.*;

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
        scoreMotor = new CANSparkMax(SCORE, MotorType.kBrushless);
        liftMotor = new CANSparkMax(LIFT, MotorType.kBrushless);
        liftEncoder = liftMotor.getEncoder();

        alignedSwitch = new DigitalInput(ALIGNED_SWITCH_CHANNEL);
        minSwitch = new DigitalInput(MIN_LIFT_CHANNEL);
        maxSwitch = new DigitalInput(MAX_LIFT_CHANNEL);
        ampIRSensor = new DigitalInput(AMP_IR_CHANNEL);

        LIFT_MOTOR.configure(liftMotor);
        SCORE_MOTOR.configure(scoreMotor);
    }

    @Override
    public boolean hasNote() {
        return !ampIRSensor.get();
    }

    @Override
    public boolean liftAtBottom() {
        return minSwitch.get();
    }

    @Override
    public boolean liftAtTop() {
        return maxSwitch.get();
    }

    @Override
    public boolean touchingAmp() {
        return alignedSwitch.get();
    }

    @Override
    public void intake() {
        scoreMotor.set(-1.0);
    }

    @Override
    public void score() {
        scoreMotor.set(1.0);
    }

    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(height);
    }

    @Override
    public void stopLift() {
        liftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Amper/Intake Speed", scoreMotor.get());
        SmartDashboard.putNumber("Amper/Lift Speed", liftMotor.get());
        SmartDashboard.putNumber("Amper/Intake Current", scoreMotor.getOutputCurrent());
        SmartDashboard.putNumber("Amper/Lift Current", liftMotor.getOutputCurrent());

        liftMotor.setVoltage(liftController.update(targetHeight.get(), liftEncoder.getPosition()));
        
        if (liftAtBottom() || liftAtTop()) {
            stopLift(); 
        }

        if (touchingAmp()) {
            score();
        }
    }
}
