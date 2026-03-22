package com.apexpathing.kinematics;

/**
 * Standard Mecanum inverse kinematics.
 */
public class MecanumKinematics extends Kinematics {
    private final double trackWidth;
    private final double wheelBase;
    private final double[] cachedSpeeds = new double[4];

    /**
     * @param trackWidth Distance between left and right wheels.
     * @param wheelBase Distance between front and back wheels.
     */
    public MecanumKinematics(double trackWidth, double wheelBase) {
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
    }

    @Override
    public double[] toWheelSpeeds(ChassisSpeeds speeds) {
        double vx = speeds.translation.getXComponent();
        double vy = speeds.translation.getYComponent();
        double omega = speeds.omega;

        double k = (trackWidth + wheelBase) / 2.0;

        // Front Left, Front Right, Back Left, Back Right
        cachedSpeeds[0] = vx - vy - k * omega;
        cachedSpeeds[1] = vx + vy + k * omega;
        cachedSpeeds[2] = vx + vy - k * omega;
        cachedSpeeds[3] = vx - vy + k * omega;

        return cachedSpeeds;
    }
}
