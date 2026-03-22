package com.apexpathing.kinematics;

/**
 * Standard Tank drive kinematics.
 */
public class TankKinematics extends Kinematics {
    private final double trackWidth;
    private final double[] cachedSpeeds = new double[2];

    /**
     * @param trackWidth Distance between left and right wheels.
     */
    public TankKinematics(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    @Override
    public double[] toWheelSpeeds(ChassisSpeeds speeds) {
        // Ignores vy translation for tank drive
        double vx = speeds.translation.getXComponent();
        double omega = speeds.omega;

        cachedSpeeds[0] = vx - (trackWidth / 2.0) * omega;
        cachedSpeeds[1] = vx + (trackWidth / 2.0) * omega;

        return cachedSpeeds;
    }
}
