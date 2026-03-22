package com.apexpathing.kinematics;

/**
 * Utility class to manage and switch between different kinematics implementations.
 */
public class KinematicsSwitcher {
    private static Kinematics activeKinematics;

    /**
     * Sets the active drive type kinematics.
     * @param kinematics The kinematics implementation to use.
     */
    public static void setDriveType(Kinematics kinematics) {
        activeKinematics = kinematics;
    }

    /**
     * Gets the active kinematics implementation.
     * @return The current Kinematics object.
     * @throws IllegalStateException if kinematics haven't been set.
     */
    public static Kinematics get() {
        if (activeKinematics == null) {
            throw new IllegalStateException("Kinematics must be set using KinematicsSwitcher.setDriveType() before use.");
        }
        return activeKinematics;
    }
}
