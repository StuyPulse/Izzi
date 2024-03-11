package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveOrchestra extends SubsystemBase {

    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFX motor3;
    private final TalonFX motor4;

    Orchestra orchestra = new Orchestra();

    public SwerveOrchestra() {
        motor1 = new TalonFX(Ports.Swerve.FrontRight.DRIVE);
        motor2 = new TalonFX(Ports.Swerve.FrontLeft.DRIVE);
        motor3 = new TalonFX(Ports.Swerve.BackRight.DRIVE);
        motor4 = new TalonFX(Ports.Swerve.BackLeft.DRIVE);

        orchestra.addInstrument(motor1);
        orchestra.addInstrument(motor2);
        orchestra.addInstrument(motor3);
        orchestra.addInstrument(motor4);

        orchestra.play();
        orchestra.pause();
        orchestra.stop();

        var status = orchestra.loadMusic("track.chrp");
    }
}