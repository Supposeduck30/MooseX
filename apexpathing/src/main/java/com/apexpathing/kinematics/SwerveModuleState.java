package com.apexpathing.kinematics;

import com.apexpathing.util.math.Vector;

/**
 * Represents a swerve module's target state.
 */
public class SwerveModuleState {
    public final Vector velocity;

    /**
     * Primary constructor using a velocity vector to avoid unnecessary trig.
     * @param velocityVector The velocity vector of the module.
     */
    public SwerveModuleState(Vector velocityVector) {
        this.velocity = velocityVector;
    }

    /**
     * Static factory method to create a SwerveModuleState from speed and angle.
     * @param speed Target speed.
     * @param angleRadians Target angle in radians.
     * @return A new SwerveModuleState.
     */
    public static SwerveModuleState fromPolar(double speed, double angleRadians) {
        return new SwerveModuleState(Vector.Companion.fromPolar(speed, angleRadians));
    }

    public double getSpeed() {
        return velocity.magnitude();
    }

    public double getAngle() {
        return velocity.theta();
    }

    @Override
    public String toString() {
        return String.format("SwerveModuleState(velocity=%s)", velocity.toString());
    }
}
