package com.apexpathing;

import static org.junit.Assert.*;
import org.junit.Test;
import com.apexpathing.geometry.Vector;
import com.apexpathing.kinematics.*;

public class KinematicsTest {

    @Test
    public void testMecanumKinematics() {
        MecanumKinematics mecanum = new MecanumKinematics(10, 10);
        ChassisSpeeds speeds = new ChassisSpeeds(new Vector(1, 0), 0);
        double[] wheelSpeeds = (double[]) mecanum.toWheelSpeeds(speeds);

        // Pure forward: all wheels same speed
        for (double s : wheelSpeeds) {
            assertEquals(1.0, s, 1e-9);
        }

        speeds = new ChassisSpeeds(new Vector(0, 1), 0);
        wheelSpeeds = (double[]) mecanum.toWheelSpeeds(speeds);
        // Pure strafe left (positive Y): FL=-1, FR=1, BL=1, BR=-1
        assertEquals(-1.0, wheelSpeeds[0], 1e-9);
        assertEquals(1.0, wheelSpeeds[1], 1e-9);
        assertEquals(1.0, wheelSpeeds[2], 1e-9);
        assertEquals(-1.0, wheelSpeeds[3], 1e-9);
    }

    @Test
    public void testSwerveKinematics() {
        Vector[] offsets = {
            new Vector(5, 5),   // FL
            new Vector(5, -5),  // FR
            new Vector(-5, 5),  // BL
            new Vector(-5, -5)  // BR
        };
        SwerveKinematics swerve = new SwerveKinematics(offsets);
        ChassisSpeeds speeds = new ChassisSpeeds(new Vector(1, 0), 0);
        SwerveModuleState[] states = (SwerveModuleState[]) swerve.toWheelSpeeds(speeds);

        for (SwerveModuleState state : states) {
            assertEquals(1.0, state.getSpeed(), 1e-9);
            assertEquals(0.0, state.getAngle(), 1e-9);
        }
    }

    @Test
    public void testTankKinematics() {
        TankKinematics tank = new TankKinematics(10);
        ChassisSpeeds speeds = new ChassisSpeeds(new Vector(1, 0), 0.1);
        double[] wheelSpeeds = (double[]) tank.toWheelSpeeds(speeds);

        // left = vx - (trackWidth/2)*omega = 1 - 5*0.1 = 0.5
        // right = vx + (trackWidth/2)*omega = 1 + 5*0.1 = 1.5
        assertEquals(0.5, wheelSpeeds[0], 1e-9);
        assertEquals(1.5, wheelSpeeds[1], 1e-9);
    }
}
