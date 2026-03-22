package com.apexpathing.kinematics;

/**
 * Base interface defining the contract for all drivetrain kinematics.
 */
public interface Kinematics {
    /**
     * Translates robot-relative ChassisSpeeds into individual wheel speeds or states.
     * @param speeds The target robot-relative speeds.
     * @return An array of wheel speeds (double[]) or module states (SwerveModuleState[]).
     */
    Object toWheelSpeeds(ChassisSpeeds speeds);
}
