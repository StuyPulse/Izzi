package com.stuypulse.robot.subsystems.swerve.modules;

import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {

    private final String id;
    
    private final Translation2d offset;

    public SwerveModuleState targetState;

    protected final Controller driveController;
    protected final AngleController angleController;

    public SwerveModule(String id, Translation2d offset) {
        this.id = id;
        this.offset = offset;

        targetState = new SwerveModuleState();

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        angleController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_TURNING));
    }

    public String getId() {
        return this.id;
    }

    public Translation2d getModuleOffset() {
        return this.offset;
    }

    public abstract double getVelocity();
    public abstract Rotation2d getAngle();
    public abstract SwerveModulePosition getModulePosition();

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public void setState(SwerveModuleState state) {
        targetState = state;
    }

    protected abstract void setVoltageImpl(double driveVoltage, double turnVoltage);

    @Override
    public void periodic() {
        driveController.update(
            targetState.speedMetersPerSecond,
            getVelocity()
        );

        angleController.update(
            Angle.fromRotation2d(targetState.angle),
            Angle.fromRotation2d(getAngle())
        );

        setVoltageImpl(driveController.getOutput(), angleController.getOutput());

        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Drive Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Turn Voltage", angleController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Angle Error", angleController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId()+ "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Speed", getVelocity());
    }
}
    

