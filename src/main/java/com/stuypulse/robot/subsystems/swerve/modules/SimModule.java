package com.stuypulse.robot.subsystems.swerve.modules;import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Controller.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Controller.Turn;
import com.stuypulse.robot.util.PositionVelocitySystem;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimModule extends AbstractSwerveModule {
    private String id;
    
    private Translation2d offset;
    
    private SwerveModuleState state;
    
    private Controller driveController;
    private AngleController angleController;

    private LinearSystemSim<N2, N1, N2> driveSim;
    private LinearSystemSim<N2, N1, N1> turnSim;

    public SimModule(String id, Translation2d offset) {
        this.id = id;
        this.offset = offset;

        this.state = new SwerveModuleState();

        this.driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        this.angleController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_TURNING));

        this.driveSim = new PositionVelocitySystem(Drive.kV, Drive.kA).getSim();
        this.turnSim = new LinearSystemSim<N2,N1,N1>(LinearSystemId.identifyPositionSystem(Turn.kV.get(), Turn.kA.get()));
    }

    @Override
    public double getVelocity() {
        return driveSim.getOutput(1);
    }

    public double getDistance() {
        return driveSim.getOutput(0);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turnSim.getOutput(0));
    }

    @Override
    public String getId() {
        return this.id;
    }

    @Override
    public SwerveModuleState getState() {
        return state;
    }

    @Override
    public Translation2d getModuleOffset() {
        return offset;
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    @Override
    public void setState(SwerveModuleState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        driveController.update(
            state.speedMetersPerSecond,
            getVelocity()
        );

        angleController.update(
            Angle.fromRotation2d(state.angle),
            Angle.fromRotation2d(getAngle())
        );

        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Drive Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Turn Voltage", angleController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Target Angle", state.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Target Speed", state.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Speed", getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        driveSim.setInput(driveController.getOutput());
        driveSim.update(Settings.DT);

        turnSim.setInput(angleController.getOutput());
        turnSim.update(Settings.DT);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
        ));
    }
    
}
