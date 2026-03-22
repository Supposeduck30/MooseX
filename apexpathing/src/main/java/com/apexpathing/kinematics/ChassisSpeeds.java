package com.apexpathing.kinematics;

import com.apexpathing.geometry.Vector;

/**
 * A data class representing robot-relative speeds.
 */
public class ChassisSpeeds {
    public final Vector translation;
    public final double omega;

    /**
     * @param translation Robot-relative translation (vx, vy).
     * @param omega Robot-relative rotation (rad/s).
     */
    public ChassisSpeeds(Vector translation, double omega) {
        this.translation = translation;
        this.omega = omega;
    }

    /**
     * Converts field-relative speeds to robot-relative speeds.
     * @param fieldSpeeds Translation in field-relative coordinates.
     * @param robotHeading Current robot heading in radians.
     * @return A new ChassisSpeeds object.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(Vector fieldSpeeds, double robotHeading) {
        // Transform field velocity to robot-relative velocity
        // v_robot = v_field.rotate(-robotHeading)
        Vector robotSpeeds = fieldSpeeds.copy().rotateVector(-robotHeading);
        return new ChassisSpeeds(robotSpeeds, 0.0);
    }

    @Override
    public String toString() {
        return String.format("ChassisSpeeds(vx=%.3f, vy=%.3f, omega=%.3f)",
            translation.getXComponent(), translation.getYComponent(), omega);
    }
}
