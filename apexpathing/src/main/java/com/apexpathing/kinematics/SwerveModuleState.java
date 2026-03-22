package com.apexpathing.kinematics;

import com.apexpathing.geometry.Vector;

/**
 * A data class representing a single swerve module's state.
 */
public class SwerveModuleState {
    public final Vector velocity;

    /**
     * Primary rectangular constructor to avoid unnecessary trig.
     * @param velocity Velocity vector for the module.
     */
    public SwerveModuleState(Vector velocity) {
        this.velocity = velocity;
    }

    /**
     * @return Magnitude of the velocity vector.
     */
    public double getSpeed() {
        return velocity.getMagnitude();
    }

    /**
     * @return Angle of the velocity vector in radians.
     */
    public double getAngle() {
        return velocity.getTheta();
    }

    /**
     * Static factory from polar coordinates.
     * @param speed Linear speed.
     * @param angleRadians Wheel angle in radians.
     * @return A new SwerveModuleState object.
     */
    public static SwerveModuleState fromPolar(double speed, double angleRadians) {
        return new SwerveModuleState(Vector.Companion.fromPolar(speed, angleRadians));
    }

    @Override
    public String toString() {
        return String.format("SwerveModuleState(speed=%.3f, angle=%.3f)", getSpeed(), getAngle());
    }
}
