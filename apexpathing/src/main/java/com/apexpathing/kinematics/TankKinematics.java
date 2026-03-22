package com.apexpathing.kinematics;

/**
 * Standard Tank drive inverse kinematics.
 */
public class TankKinematics implements Kinematics {
    private final double trackWidth;
    private final double[] wheelSpeeds = new double[2];

    /**
     * @param trackWidth Distance between left and right wheels.
     */
    public TankKinematics(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    @Override
    public double[] toWheelSpeeds(ChassisSpeeds speeds) {
        // Tank drive ignores Y velocity
        double vx = speeds.translation.x();
        double omega = speeds.omega;

        wheelSpeeds[0] = vx - (trackWidth / 2.0) * omega;
        wheelSpeeds[1] = vx + (trackWidth / 2.0) * omega;

        return wheelSpeeds;
    }
}
