package com.stuypulse.robot.subsystems.swerve.modules;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.robot.util.PositionVelocitySystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class SimModule extends SwerveModule {
    private final LinearSystemSim<N2, N1, N2> driveSim;
    private final LinearSystemSim<N2, N1, N1> turnSim;

    public SimModule(String id, Translation2d offset) {
        super(id, offset);
        driveSim = new PositionVelocitySystem(Drive.kV, Drive.kA).getSim();
        turnSim = new LinearSystemSim<N2,N1,N1>(LinearSystemId.identifyPositionSystem(Turn.kV.get(), Turn.kA.get()));
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
    public void setVoltageImpl() {
        driveSim.setInput(this.driveController.getOutput());
        turnSim.setInput(this.angleController.getOutput());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
        ));
    }

    @Override
    public void simulationPeriodic() {
        driveSim.update(Settings.DT);
        turnSim.update(Settings.DT);
    }
}
