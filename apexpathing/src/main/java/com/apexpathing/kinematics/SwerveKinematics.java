package com.apexpathing.kinematics;

import com.apexpathing.geometry.Vector;

/**
 * Standard Swerve inverse kinematics using 4 module positions.
 */
public class SwerveKinematics extends Kinematics {
    private final Vector[] moduleOffsets;
    private final SwerveModuleState[] states;

    /**
     * @param moduleOffsets Positions of modules relative to robot center (FL, FR, BL, BR).
     */
    public SwerveKinematics(Vector[] moduleOffsets) {
        if (moduleOffsets.length != 4) {
            throw new IllegalArgumentException("SwerveKinematics requires 4 module positions.");
        }
        this.moduleOffsets = moduleOffsets;
        this.states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(new Vector(0, 0));
        }
    }

    @Override
    public SwerveModuleState[] toWheelSpeeds(ChassisSpeeds speeds) {
        double vx = speeds.translation.getXComponent();
        double vy = speeds.translation.getYComponent();
        double omega = speeds.omega;

        for (int i = 0; i < 4; i++) {
            // v_i = v_robot + omega x r_i
            // vx_i = vx - omega * ry_i
            // vy_i = vy + omega * rx_i
            double moduleVX = vx - omega * moduleOffsets[i].getYComponent();
            double moduleVY = vy + omega * moduleOffsets[i].getXComponent();

            states[i].velocity.setX(moduleVX);
            states[i].velocity.setY(moduleVY);
        }

        return states;
    }
}
