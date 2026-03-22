package com.apexpathing.kinematics;

/**
 * Utility class to manage global drivetrain kinematics.
 */
public class KinematicsSwitcher {
    private static Kinematics instance;

    /**
     * Sets the active kinematics implementation for the current OpMode.
     * @param kinematics Implementation (Mecanum, Swerve, Tank).
     */
    public static void setDriveType(Kinematics kinematics) {
        instance = kinematics;
    }

    /**
     * @return The currently configured kinematics.
     */
    public static Kinematics get() {
        if (instance == null) {
            throw new IllegalStateException("Kinematics has not been initialized. " +
                "Call KinematicsSwitcher.setDriveType() before following a trajectory.");
        }
        return instance;
    }
}
