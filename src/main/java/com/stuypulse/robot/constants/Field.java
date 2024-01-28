package com.stuypulse.robot.constants;

import java.util.ArrayList;

import com.stuypulse.robot.util.Fiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This interface stores information about the field elements.
 */
public interface Field {

    double NOTE_LENGTH = Units.inchesToMeters(14.0);

    /*** APRILTAGS ***/

    double FIDUCIAL_SIZE = Units.inchesToMeters(6.125);

    Fiducial FIDUCIALS[] = {
        // 2024 Field Fiducial Layout
        new Fiducial(1,  new Pose3d(new Translation3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new Fiducial(2,  new Pose3d(new Translation3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new Fiducial(3,  new Pose3d(new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new Fiducial(4,  new Pose3d(new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new Fiducial(5,  new Pose3d(new Translation3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new Fiducial(6,  new Pose3d(new Translation3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new Fiducial(7,  new Pose3d(new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new Fiducial(8,  new Pose3d(new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new Fiducial(9,  new Pose3d(new Translation3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new Fiducial(10, new Pose3d(new Translation3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new Fiducial(11, new Pose3d(new Translation3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(300)))),
        new Fiducial(12, new Pose3d(new Translation3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new Fiducial(13, new Pose3d(new Translation3d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new Fiducial(14, new Pose3d(new Translation3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new Fiducial(15, new Pose3d(new Translation3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new Fiducial(16, new Pose3d(new Translation3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(240)))),
    };

    public static boolean isValidFiducial(int fid) {
        for(Fiducial fiducial : FIDUCIALS) { 
            if (fiducial.getFID() == fid) { 
                return true;
            }
        }
        return false;
    }
    
    public static Fiducial[] getFiducialLayout(int... fids) {
        ArrayList<Fiducial> fiducials = new ArrayList<Fiducial>();

        for (int fid : fids) {
            for (Fiducial fiducial : FIDUCIALS) {
                if (fiducial.getFID() == fid) { 
                    fiducials.add(fiducial);
                }
            }
        }
                
        Fiducial[] fiducials_array = new Fiducial[fiducials.size()];
        return fiducials.toArray(fiducials_array);
    }

    public static double[] getLayoutAsDoubleArray(Fiducial[] fiducials) {
        double[] layout = new double[fiducials.length * 7];
        for (int i = 0; i < fiducials.length; i++) {
            layout[i * 7 + 0] = fiducials[i].getFID();
            layout[i * 7 + 1] = fiducials[i].getLocation().getX();
            layout[i * 7 + 2] = fiducials[i].getLocation().getY();
            layout[i * 7 + 3] = fiducials[i].getLocation().getZ();
            layout[i * 7 + 4] = fiducials[i].getLocation().getRotation().getX();
            layout[i * 7 + 5] = fiducials[i].getLocation().getRotation().getY();
            layout[i * 7 + 6] = fiducials[i].getLocation().getRotation().getZ();
        }
        return layout;
    }

    public static Fiducial getFiducial(int fid) {
        for (Fiducial fiducial : FIDUCIALS) {
            if (fiducial.getFID() == fid) {
                return fiducial;
            }
        }
        return null;
    }

    /*** SPEAKER ***/

    Pose2d SPEAKER_POSES[] = {
        getFiducial(7).getLocation().toPose2d(), // BLUE CENTER
        getFiducial(4).getLocation().toPose2d(), // RED CENTER
    };

    public static Pose2d getAllianceSpeakerPose() {
        boolean isBlue = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        return SPEAKER_POSES[isBlue ? 0 : 1];
    }

    double SPEAKER_OPENING_X = Units.inchesToMeters(13.6);
}
