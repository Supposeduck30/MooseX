package com.apexpathing.kinematics;

import com.apexpathing.util.math.Vector;

/**
 * Standard Swerve inverse kinematics.
 */
public class SwerveKinematics implements Kinematics {
    private final Vector[] moduleLocations;
    private final SwerveModuleState[] states;

    /**
     * @param moduleLocations Physical locations of the swerve modules relative to the robot center.
     */
    public SwerveKinematics(Vector[] moduleLocations) {
        this.moduleLocations = moduleLocations;
        this.states = new SwerveModuleState[moduleLocations.length];
        for (int i = 0; i < moduleLocations.length; i++) {
            states[i] = new SwerveModuleState(new Vector(0, 0));
        }
    }

    @Override
    public SwerveModuleState[] toWheelSpeeds(ChassisSpeeds speeds) {
        for (int i = 0; i < moduleLocations.length; i++) {
            // v = v_trans + omega x r
            // vx = v_trans_x - omega * r_y
            // vy = v_trans_y + omega * r_x
            double vx = speeds.translation.x() - speeds.omega * moduleLocations[i].y();
            double vy = speeds.translation.y() + speeds.omega * moduleLocations[i].x();

            states[i].velocity.setX(vx);
            states[i].velocity.setY(vy);
        }
        return states;
    }
}
