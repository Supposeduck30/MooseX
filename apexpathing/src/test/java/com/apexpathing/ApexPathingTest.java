package com.apexpathing;

import static org.junit.Assert.*;
import org.junit.Test;
import com.apexpathing.util.math.Vector;
import com.apexpathing.follower.QuinticHermiteSpline;
import com.apexpathing.util.math.Pose;

public class ApexPathingTest {

    @Test
    public void testPoseFunctions() {
        Pose myTestPose = new Pose(10.0, 10.0);
        assertEquals(10.0, myTestPose.withReflectedY().y(), 0.0);
        assertEquals(-10.0, myTestPose.withReflectedY().x(), 0.0);
    }

    @Test
    public void testVectorMath() {
        Vector v1 = new Vector(1, 2);
        Vector v2 = new Vector(3, 4);

        Vector v3 = v1.plus(v2);
        assertEquals(4, v3.x(), 1e-9);
        assertEquals(6, v3.y(), 1e-9);

        Vector v4 = v1.rotateVec(Math.PI / 2);
        assertEquals(-2, v4.x(), 1e-9);
        assertEquals(1, v4.y(), 1e-9);
    }

    @Test
    public void testQuinticHermiteSpline() {
        Vector p0 = new Vector(0, 0);
        Vector v0 = new Vector(1, 0);
        Vector a0 = new Vector(0, 0);

        Vector p1 = new Vector(1, 1);
        Vector v1 = new Vector(0, 1);
        Vector a1 = new Vector(0, 0);

        QuinticHermiteSpline spline = new QuinticHermiteSpline(p0, v0, a0, p1, v1, a1);

        Vector start = spline.getPoint(0);
        assertEquals(0, start.x(), 1e-9);
        assertEquals(0, start.y(), 1e-9);

        Vector end = spline.getPoint(1);
        assertEquals(1, end.x(), 1e-9);
        assertEquals(1, end.y(), 1e-9);

        Vector startVel = spline.getVelocity(0);
        assertEquals(1, startVel.x(), 1e-9);
        assertEquals(0, startVel.y(), 1e-9);

        Vector endVel = spline.getVelocity(1);
        assertEquals(0, endVel.x(), 1e-9);
        assertEquals(1, endVel.y(), 1e-9);
    }
}
