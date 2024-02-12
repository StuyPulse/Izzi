package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics kinematics;
    
    public SwerveSetpointGenerator(final SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    @FunctionalInterface
    private interface Function2d {
        public double f(Translation2d vector);
    }

    private class RegulaFalsiState {
        public int iterationsLeft;
        public Translation2d leftEndInput;
        public Translation2d rightEndInput;
        public double leftEndOutput;
        public double rightEndOutput;

        public RegulaFalsiState(int iterationsLeft, Translation2d leftEndInput, Translation2d rightEndInput, double leftEndOutput, double rightEndOutput) {
            this.iterationsLeft = iterationsLeft;
            this.leftEndInput = leftEndInput;
            this.rightEndInput = rightEndInput;
            this.leftEndOutput = leftEndOutput;
            this.rightEndOutput = rightEndOutput;
        }
    }

    /**
     * @param a first number
     * @param b second number
     * @return whether the two numbers are within 1e-6 of each other.
     */
    private boolean toleranceEquals(double a, double b) {
        return Math.abs(a - b) < 1e-2; 
    }

    /**
     * If the given angle is more than 180 degrees from the reference angle,
     * revolve it the other way so the delta is minimized.
     * @param angle the angle to unwrap (radians)
     * @param reference the angle to check against (radians)
     * @return the unwrapped angle.
     */
    private double unwrapAngle(double angle, double reference) {
        double delta = angle - reference;
        if (delta > Math.PI) {
            return angle - 2 * Math.PI;
        }
        else if (delta < -Math.PI) {
            return angle + 2 * Math.PI;
        }
        else {
            return angle;
        }
    }

    private double findRoot(Function2d func, RegulaFalsiState state) {        
        double interpolant = 0.0;
        while(state.iterationsLeft > 0 && toleranceEquals(state.leftEndOutput, state.rightEndOutput)) {
            double interpolantTerm = -state.leftEndOutput / (state.rightEndOutput - state.leftEndOutput);
            interpolantTerm = Math.max(0.0, Math.min(1.0, interpolantTerm)); // keep within [0, 1]
            
            Translation2d rootGuess = state.rightEndInput.minus(state.leftEndInput).times(interpolantTerm).plus(state.leftEndInput);
            double rootGuessOutput = func.f(rootGuess); 

            if (rootGuessOutput < 0) {
                // Since we undershot, we'll add the fraction of the remaining interpolant 
                // term to our current interpolant term.
                interpolant = interpolant * (1 - interpolantTerm) + interpolantTerm;
                state.leftEndInput = rootGuess;
                state.leftEndOutput = rootGuessOutput;
            }
            else { 
                // Since we overshot, the interpolant term is the fraction of the previous
                // interpolant that our next interpolant must be.
                interpolant *= interpolantTerm;
                state.rightEndInput = rootGuess;
                state.rightEndOutput = rootGuessOutput; 
            }
            --state.iterationsLeft;
        }

        return interpolant;
    }
    
    // Gets the Max Drive Interpolant
    private double getMaxDriveInterpolant(double maxVelocityStep, RegulaFalsiState state) {
        double deltaVelocity = state.rightEndOutput - state.leftEndOutput;
        // If the amount we want to accelerate is less than the maximum feasible acceleration,
        // there is no need to calculate an interpolant. We'll just accelerate all the way.
        if (Math.abs(deltaVelocity) <= maxVelocityStep) {
            return 1.0;
        }
        
        // nextVelocity is the maximum next achievable velocity in accordance with our kinematics
        // constraints. currentToNextVelocity returns the delta of that next velocity to some given
        // velocity. Solving for the roots of this equation gives us the velocity vector of our setpoint.
        double nextVelocity = state.leftEndOutput + Math.signum(deltaVelocity) * maxVelocityStep;
        Function2d currentToNextVelocity = (Translation2d velocity) -> velocity.getNorm() - nextVelocity;
        
        // We are essentially evaulating currentToNextVelocity at the current inputs. However,
        // currentToNextVelocity(state.leftEndInput) is the same as the current left end output
        // minus the next velocity (same for right end input/output). 
        state.leftEndOutput -= nextVelocity;
        state.rightEndOutput -= nextVelocity;
        return findRoot(currentToNextVelocity, state);
    }

    private double getMaxTurnInterpolant(double maxAngleStep, RegulaFalsiState state) {
        state.rightEndOutput = unwrapAngle(state.rightEndOutput, state.leftEndOutput);
        double deltaAngle = state.rightEndOutput - state.leftEndOutput;
        // If the amount we want to turn is less than the maximum feasible turn, there is
        // no need to calculate an interpolant. We'll just turn all the way.
        if (Math.abs(deltaAngle) <= maxAngleStep) {
            return 1.0;
        }
        
        // nextAngle is the maximum next achievable angle, currentToNextAngle returns
        // the delta of that next velocity to some given velocity. Solving for the roots of
        // this equation gives us the velocity vector of our setpoint.
        double nextAngle = state.leftEndOutput + Math.signum(deltaAngle) * maxAngleStep;
        Function2d currentToNextAngle = (Translation2d velocity) -> 
            unwrapAngle(velocity.getAngle().getRadians(), state.leftEndOutput) - nextAngle;
        
        // We are essentially evaulating currentToNextVelocity at the current inputs. However,
        // currentToNextVelocity(state.leftEndInput) is the same as the current left end output
        // minus the next velocity (same for right end input/output). 
        state.leftEndOutput -= nextAngle;
        state.rightEndOutput -= nextAngle;
        return findRoot(currentToNextAngle, state);
    }

    public ChassisSpeeds generateSetpoint(ChassisSpeeds prevSetpoint, ChassisSpeeds desiredState) {
        SwerveConstraints constraints = Swerve.CONSTRAINTS;
        SwerveModuleState[] prevModuleStates = kinematics.toSwerveModuleStates(prevSetpoint);
        SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(desiredState);

        // Enforce drive velocity limits
        // SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, constraints.maxDriveVel);
        // desiredState = kinematics.toChassisSpeeds(desiredModuleStates);

        // Get minimum turn interpolant across all modules i.e. enforce turn velocity limits   
        int maxTurnIterations = 8;
        double minTurnInterpolant = 1.0;
        for (int i = 0; i < prevModuleStates.length; i++) {
            Translation2d prevVel = new Translation2d(prevModuleStates[i].speedMetersPerSecond, prevModuleStates[i].angle);
            Translation2d desiredVel = new Translation2d(desiredModuleStates[i].speedMetersPerSecond, desiredModuleStates[i].angle);
            
            double turnInterpolant = getMaxTurnInterpolant(
                constraints.maxTurnVel * Settings.DT,
                new RegulaFalsiState(
                    maxTurnIterations,
                    prevVel,
                    desiredVel,
                    prevVel.getAngle().getRadians(),
                    desiredVel.getAngle().getRadians()
                )
            );
            minTurnInterpolant = Math.min(minTurnInterpolant, turnInterpolant);
        }

        // Get minimum drive interpolant across all modules i.e. enforce drive acceleration limits
        int maxDriveIterations = 11;
        double minDriveInterpolant = 1.0;
        for (int i = 0; i < prevModuleStates.length; i++) {
            Translation2d prevVel = new Translation2d(prevModuleStates[i].speedMetersPerSecond, prevModuleStates[i].angle);
            Translation2d desiredVel = new Translation2d(desiredModuleStates[i].speedMetersPerSecond, desiredModuleStates[i].angle);

            double driveInterpolant = getMaxDriveInterpolant(
                constraints.maxDriveAccel * Settings.DT,
                new RegulaFalsiState(
                    maxDriveIterations,
                    prevVel,
                    desiredVel,
                    prevVel.getNorm(),
                    desiredVel.getNorm()
                )
            );
            minDriveInterpolant = Math.min(minDriveInterpolant, driveInterpolant);
        } 

        SmartDashboard.putNumber("Swerve/Turn S", minTurnInterpolant);
        SmartDashboard.putNumber("Swerve/Drive S", minDriveInterpolant);

        double dx = desiredState.vxMetersPerSecond - prevSetpoint.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.vyMetersPerSecond;
        double dtheta = desiredState.omegaRadiansPerSecond - prevSetpoint.omegaRadiansPerSecond;
        ChassisSpeeds setpoint = new ChassisSpeeds(
            prevSetpoint.vxMetersPerSecond + minDriveInterpolant * dx,
            prevSetpoint.vyMetersPerSecond + minDriveInterpolant * dy,
            prevSetpoint.omegaRadiansPerSecond + minTurnInterpolant * dtheta
        );

        System.out.println("LOG: " + desiredState.vxMetersPerSecond + " " + desiredState.vyMetersPerSecond + " " + desiredState.omegaRadiansPerSecond);

        return setpoint;
    }
}
