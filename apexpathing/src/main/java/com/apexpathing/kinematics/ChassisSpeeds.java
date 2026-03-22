package com.apexpathing.kinematics;

import com.apexpathing.util.math.Vector;

/**
 * Represents robot-relative velocities.
 */
public class ChassisSpeeds {
    public final Vector translation;
    public double omega;

    public ChassisSpeeds(Vector translation, double omega) {
        this.translation = translation;
        this.omega = omega;
    }

    public ChassisSpeeds(double vx, double vy, double omega) {
        this(new Vector(vx, vy), omega);
    }

    /**
     * Converts field-relative speeds to robot-relative speeds.
     * @param fieldSpeeds Vector representing field-relative translation speeds.
     * @param robotHeading Current robot heading in radians.
     * @return Robot-relative ChassisSpeeds.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(Vector fieldSpeeds, double robotHeading) {
        return new ChassisSpeeds(fieldSpeeds.rotateVec(-robotHeading), 0.0);
    }

    /**
     * Converts field-relative speeds to robot-relative speeds.
     * @param fieldSpeeds Vector representing field-relative translation speeds.
     * @param robotHeading Current robot heading in radians.
     * @param fieldOmega Field-relative rotation speed (usually same as robot-relative).
     * @return Robot-relative ChassisSpeeds.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(Vector fieldSpeeds, double robotHeading, double fieldOmega) {
        return new ChassisSpeeds(fieldSpeeds.rotateVec(-robotHeading), fieldOmega);
    }

    @Override
    public String toString() {
        return String.format("ChassisSpeeds(translation=%s, omega=%.3f)", translation.toString(), omega);
    }
}
