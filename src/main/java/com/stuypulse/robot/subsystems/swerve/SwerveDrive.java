/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.Derivative;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.odometry.SwerveModuleOdometry;
import com.stuypulse.robot.subsystems.swerve.modules.KrakenSwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.SwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.SwerveModuleSim;
import com.stuypulse.robot.util.FollowPathPointSpeakerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

/*
 *  Fields:
 *    - swerveModules:  AbstractSwerveModule...
 *    - kinematics:  SwerveDriveKinematics
 *    - gyro: AHRS
 *    - modules2D: FieldObject2D[]
 *
 *   Tasks:
 *    - drive
 *    - followDrive
 *    - trackingDrive
 *    - aligning (Trap, Speaker, Amp)
 *    - GTADrive (shooting while driving)
 *
 *   Methods:
 *    + singleton
 *    + initFieldObject(Field2D field): void
 *    + getModulePositions(): Translation2D[]
 *    + getModuleStates(): SwerveModuleStates[]
 *    + getModuleOffsets(): Rotation2D[]
 *    + getChassisSpeeds(): ChassisSpeed[]
 *    + setModuleStates(SwerveModuleState... states): void
 *    + setChassisSpeed(ChassisSpeed): void
 *    + drive(double, Rotation2D)
 *    + stop(double, Rotation2D)
 *
 *  SwerveDrive.java
 *   Methods:
 *    - getGyroAngle(): Rotation2D
 *    - getGyroYaw(): Rotation2D
 *    - getGyroPitch(): Rotation2D
 *    - getGyroRoll(): Rotation2D
 *    - getKinematics(): SwerveDriveKinematics
 *    + periodic(): void
 *
 *
 */
public class SwerveDrive extends SubsystemBase {

    private static final SwerveDrive instance;

    static {
        if (Robot.ROBOT == RobotType.IZZI) {
            instance = new SwerveDrive(
                // new SwerveModuleImpl(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.DRIVE, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER),
                // new SwerveModuleImpl(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.DRIVE, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER),
                // new SwerveModuleImpl(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.DRIVE, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER),
                // new SwerveModuleImpl(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.DRIVE, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER)
                new KrakenSwerveModule(FrontRight.ID, FrontRight.MODULE_OFFSET, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER),
                new KrakenSwerveModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER),
                new KrakenSwerveModule(BackLeft.ID, BackLeft.MODULE_OFFSET, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER),
                new KrakenSwerveModule(BackRight.ID, BackRight.MODULE_OFFSET, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER)
            );
        } else if (Robot.ROBOT == RobotType.TUMBLER) {
            instance = new SwerveDrive(KrakenSwerveModule.getTumblerModules());
        } else {
            instance = new SwerveDrive(
                new SwerveModuleSim(FrontRight.ID, FrontRight.MODULE_OFFSET),
                new SwerveModuleSim(FrontLeft.ID, FrontLeft.MODULE_OFFSET),
                new SwerveModuleSim(BackLeft.ID, BackLeft.MODULE_OFFSET),
                new SwerveModuleSim(BackRight.ID, BackRight.MODULE_OFFSET)
            );
        }     
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    /*** PATH FOLLOWING ***/

    public Command followPathCommand(String pathName) {
        return followPathCommand(PathPlannerPath.fromPathFile(pathName));
    }

    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathHolonomic(
            path,
            () -> Odometry.getInstance().getPose(),
            this::getChassisSpeeds,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                Motion.XY,
                Motion.THETA,
                Settings.Swerve.MAX_MODULE_SPEED,
                Math.hypot(Settings.Swerve.LENGTH, Settings.Swerve.WIDTH),
                new ReplanningConfig(false, false)
            ),
            () -> false,
            this
        );
    }

    public Command followPathWithSpeakerAlignCommand(PathPlannerPath path) {
        return new FollowPathPointSpeakerCommand(
            path, 
            () -> Odometry.getInstance().getPose(), 
            this::getChassisSpeeds, 
            this::setChassisSpeeds, 
            new PPHolonomicDriveController(
                Motion.XY, 
                Motion.THETA, 
                0.02, 
                Settings.Swerve.MAX_MODULE_SPEED, 
                Math.hypot(Settings.Swerve.LENGTH, Settings.Swerve.WIDTH)),
            new ReplanningConfig(false, false),
            () -> false,
            this
        );
    }

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
    private final FieldObject2d[] modules2D;
    private final IStream jerk;
    // gyro cant do too many readings in one tick
    private double currentJerk;

    private final StructArrayPublisher<SwerveModuleState> statesPub;

    /**
     * Creates a new Swerve Drive using the provided modules
     *
     * @param modules the modules to use
     */
    protected SwerveDrive(SwerveModule... modules) {
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        gyro = new AHRS(SPI.Port.kMXP);
        modules2D = new FieldObject2d[modules.length];
        
        IStream jerkX = IStream.create(() -> 9.81 * gyro.getWorldLinearAccelX()).filtered(new Derivative());
        IStream jerkY = IStream.create(() -> 9.81 * gyro.getWorldLinearAccelY()).filtered(new Derivative());
        
        jerk = IStream.create(() -> {
            currentJerk = Math.hypot(jerkX.get(), jerkY.get());
            return currentJerk;
        });

        currentJerk = 0;

        statesPub = NetworkTableInstance.getDefault()
            .getStructArrayTopic("SmartDashboard/Swerve/States", SwerveModuleState.struct).publish();
    }

    public void configureAutoBuilder() {
        Odometry odometry = Odometry.getInstance();

        AutoBuilder.configureHolonomic(
            odometry::getPose,
            odometry::reset,
            this::getChassisSpeeds,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                Swerve.Motion.XY,
                Swerve.Motion.THETA,
                Swerve.MAX_MODULE_SPEED,
                Swerve.WIDTH,
                new ReplanningConfig(true, true)),
            () -> false,
            instance
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> odometry.getField().getObject("path").setPoses(poses));
    }

    public void initFieldObject(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            modules2D[i] = field.getObject(modules[i].getId() + "-2d");
        }
    }

    /** Getters **/
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public String[] getModuleIds() {
        String[] ids = new String[modules.length];
        for (int i = 0; i < modules.length; i++) {
            ids[i] = modules[i].getId();
        }
        return ids;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] offsets = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getModulePosition();
        }
        return offsets;
    }

    public Translation2d[] getModuleOffsets() {
        Translation2d[] offsets = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getModuleOffset();
        }
        return offsets;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Setters **/
    public void setModuleStates(SwerveModuleState[] states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Provided incorrect number of states for swerve drive modules");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public void setFieldRelativeSpeeds(ChassisSpeeds chassisSpeeds) {
        setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            chassisSpeeds,
            Odometry.getInstance().getPose().getRotation()));
    }

    public void setChassisSpeeds(ChassisSpeeds robotSpeeds) {
        SmartDashboard.putNumber("Swerve/Chassis Target X", robotSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Target Y", robotSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Target Omega", robotSpeeds.omegaRadiansPerSecond);

        setModuleStates(kinematics.toSwerveModuleStates(robotSpeeds));
    }

    public void setXMode() {
        setModuleStates(
                new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(135))
                }
            );
    }

    /** Drive Functions * */
    public void drive(Vector2D velocity, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.y, -velocity.x,
            -rotation,
            Odometry.getInstance().getPose().getRotation());

        Pose2d robotVel = new Pose2d(
            Settings.DT * speeds.vxMetersPerSecond,
            Settings.DT * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
            twistVel.dx / Settings.DT,
            twistVel.dy / Settings.DT,
            twistVel.dtheta / Settings.DT
        ));
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    /** Gyro **/
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public double getForwardAccelerationGs() {
        return gyro.getWorldLinearAccelY();
    }

    public boolean isColliding() {
        return currentJerk > Settings.Swerve.COLLISION_JERK_THRESHOLD;
    }

    /**
     * Check whether a skid has ocurred.
     * 
     * @implNote Checks for deformity in the rectangle formed by the swerve
     * drive. If the dot product between pairs of any two vectors formed by
     * adjacent swerve modules is non-zero, then we have a deformed rectangle
     * (the vectors are not orthogonol).
     * 
     * @return true if skid detected, false otherwise.
     */
    public boolean skidDetected() {
        SwerveModuleOdometry[] moduleOdometries = Odometry.getInstance().getModuleOdometries();
        int n = moduleOdometries.length;
        Vector2D[] adjacentVectors = new Vector2D[n];

        for (int i = 0; i < n; i++) {
            Vector2D vector1 = new Vector2D(moduleOdometries[i].getPose().getTranslation());
            Vector2D vector2 = new Vector2D(moduleOdometries[(i + 1) % n].getPose().getTranslation());
            if (vector1.equals(vector2)) {
                return true;
            }
            adjacentVectors[i] = vector2.sub(vector1);
        }

        for (int i = 0; i < n; i++) {
            for(int j = i + 1; j < n; j++) {
                if (adjacentVectors[i].dot(adjacentVectors[j]) > Settings.Swerve.SKID_DOT_PRODUCT_THRESHOLD) {
                    return true;
                }
            }
        }

        return false;
    }
    
    @Override
    public void periodic() {
        // Odometry odometry = Odometry.getInstance();
        // Pose2d pose = odometry.getPose();
        // Rotation2d angle = pose.getRotation();

        // for (int i = 0; i < modules.length; i++) {
        //     modules2D[i].setPose(new Pose2d(
        //         pose.getTranslation().plus(modules[i].getModuleOffset().rotateBy(angle)),
        //         modules[i].getAngle().plus(angle)
        //     ));
        // }

        statesPub.set(getModuleStates());

        SmartDashboard.putNumber("Swerve/Gyro/Angle (deg)", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro/Pitch (deg)", getGyroPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro/Roll (deg)", getGyroRoll().getDegrees());

        SmartDashboard.putNumber("Swerve/Forward Acceleration  (Gs)", getForwardAccelerationGs());
        SmartDashboard.putNumber("Swerve/X Acceleration (Gs)", gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber("Swerve/Y Acceleration (Gs)", gyro.getWorldLinearAccelY());
        SmartDashboard.putNumber("Swerve/Z Acceleration (Gs)", gyro.getWorldLinearAccelZ());

        SmartDashboard.putNumber("Swerve/Chassis X Speed", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Y Speed", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Rotation", getChassisSpeeds().omegaRadiansPerSecond);

        SmartDashboard.putNumber("Swerve/Jerk", jerk.get());
        SmartDashboard.putBoolean("Swerve/Is Colliding", isColliding());
        SmartDashboard.putBoolean("Swerve/Skid Detected", skidDetected());
    }

    @Override
    public void simulationPeriodic() {
        // show gyro angle in simulation
        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond * Settings.DT));
    }
}
