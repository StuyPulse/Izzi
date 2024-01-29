package com.stuypulse.robot.subsystems.swerve.modules;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.robot.util.PositionVelocitySystem;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimModule extends SwerveModule {

    private final LinearSystemSim<N2, N1, N2> driveSim;
    private final LinearSystemSim<N2, N1, N1> turnSim;

    private final Controller driveController;
    private final AngleController angleController;

    public SimModule(String id, Translation2d offset) {
        super(id, offset);
        
        driveSim = new PositionVelocitySystem(Drive.kV, Drive.kA).getSim();
        turnSim = new LinearSystemSim<N2,N1,N1>(LinearSystemId.identifyPositionSystem(Turn.kV.get(), Turn.kA.get()));

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        angleController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_TURNING));
    }

    private double getPosition() {
        return driveSim.getOutput(0);
    }
    
    @Override
    public double getVelocity() {
        return driveSim.getOutput(1);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turnSim.getOutput(0));
    } 
  
    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveSim.getOutput(0), getAngle());
    }

    @Override
    public void periodic() {
        super.periodic();

        driveController.update(
            getTargetState().speedMetersPerSecond,
            getVelocity()
        );

        angleController.update(
            Angle.fromRotation2d(getTargetState().angle),
            Angle.fromRotation2d(getAngle())
        );

        if (Math.abs(driveController.getOutput()) < Settings.Swerve.MODULE_VELOCITY_DEADBAND.get()) {
            driveSim.setInput(0);
            turnSim.setInput(0);
        } else {
            driveSim.setInput(MathUtil.clamp(driveController.getOutput(), -12, 12));
            turnSim.setInput(MathUtil.clamp(angleController.getOutput(), -12, 12));
        }

        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Voltage", angleController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Position", getPosition());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle Error", angleController.getError().toDegrees());
    }

    @Override
    public void simulationPeriodic() {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
        ));

        driveSim.update(Settings.DT);
        turnSim.update(Settings.DT);
    }
}
