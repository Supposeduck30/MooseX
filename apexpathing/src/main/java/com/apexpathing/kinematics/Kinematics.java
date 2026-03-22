package com.apexpathing.kinematics;

/**
 * Abstract base class for all drivetrain kinematics.
 */
public abstract class Kinematics {
    /**
     * Translates a ChassisSpeeds object into wheel velocities (or states).
     * @param speeds The target robot-relative speeds.
     * @return Output from the kinematics model (e.g., wheel speeds or module states).
     */
    public abstract Object toWheelSpeeds(ChassisSpeeds speeds);
}
