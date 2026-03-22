package com.apexpathing.kinematics;

/**
 * Standard Mecanum inverse kinematics.
 */
public class MecanumKinematics implements Kinematics {
    private final double trackWidth;
    private final double wheelBase;
    private final double k;
    private final double[] wheelSpeeds = new double[4];

    /**
     * @param trackWidth Distance between left and right wheels.
     * @param wheelBase Distance between front and back wheels.
     */
    public MecanumKinematics(double trackWidth, double wheelBase) {
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.k = (trackWidth + wheelBase) / 2.0;
    }

    @Override
    public double[] toWheelSpeeds(ChassisSpeeds speeds) {
        double vx = speeds.translation.x();
        double vy = speeds.translation.y();
        double omega = speeds.omega;

        wheelSpeeds[0] = vx - vy - k * omega;
        wheelSpeeds[1] = vx + vy + k * omega;
        wheelSpeeds[2] = vx + vy - k * omega;
        wheelSpeeds[3] = vx - vy + k * omega;

        return wheelSpeeds;
    }
}
