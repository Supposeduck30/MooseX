package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Quintic Curve Test", group="Drive")
public class QuinticCurveTest extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;
    DcMotorEx fL, fR, bL, bR;

    class QuinticCoeffs {
        double a, b, c, d, e, f;
        public double getPos(double t) {
            return a*Math.pow(t,5) + b*Math.pow(t,4) + c*Math.pow(t,3) + d*Math.pow(t,2) + e*t + f;
        }
        public double getVel(double t) {
            return 5*a*Math.pow(t,4) + 4*b*Math.pow(t,3) + 3*c*Math.pow(t,2) + 2*d*t + e;
        }
    }

    @Override
    public void runOpMode() {
        // --- Initialization ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        fL = hardwareMap.get(DcMotorEx.class, "fl");
        fR = hardwareMap.get(DcMotorEx.class, "fr");
        bL = hardwareMap.get(DcMotorEx.class, "bl");
        bR = hardwareMap.get(DcMotorEx.class, "br");
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Path Definition ---
        double totalTime = 3.0;
        double midTime = totalTime / 2.0;

        // Points: Start(0,0) -> Mid(24, 0) -> End(24, 24)
        // To curve, we "inject" a velocity at the midpoint.
        double vMidX = 20; // Pushing toward the right
        double vMidY = 20; // Pushing "up" the field

        // Segment 1: Start to Mid
        QuinticCoeffs xSeg1 = fitQuintic(0, 0, 0, 24, vMidX, 0, midTime);
        QuinticCoeffs ySeg1 = fitQuintic(0, 0, 0, 0, vMidY, 0, midTime);

        // Segment 2: Mid to End
        QuinticCoeffs xSeg2 = fitQuintic(24, vMidX, 0, 24, 0, 0, midTime);
        QuinticCoeffs ySeg2 = fitQuintic(0, vMidY, 0, 24, 0, 0, midTime);

        waitForStart();
        resetRuntime();

        while (opModeIsActive() && getRuntime() < totalTime) {
            double t = getRuntime();
            pinpoint.update();

            double targetX, targetY, targetVx, targetVy;

            // --- Segment Switching Logic ---
            if (t < midTime) {
                targetX = xSeg1.getPos(t);
                targetY = ySeg1.getPos(t);
                targetVx = xSeg1.getVel(t);
                targetVy = ySeg1.getVel(t);
            } else {
                double localT = t - midTime; // Reset time for the second segment
                targetX = xSeg2.getPos(localT);
                targetY = ySeg2.getPos(localT);
                targetVx = xSeg2.getVel(localT);
                targetVy = ySeg2.getVel(localT);
            }

            // --- Heading and Drive Control ---
            double targetHeading = Math.atan2(targetVy, targetVx);
            double currX = pinpoint.getPosX(DistanceUnit.INCH);
            double currY = pinpoint.getPosY(DistanceUnit.INCH);
            double currH = pinpoint.getHeading(AngleUnit.RADIANS);

            double driveX = (targetX - currX) * 0.1 + (targetVx * 0.015);
            double driveY = (targetY - currY) * 0.1 + (targetVy * 0.015);
            double driveR = AngleUnit.normalizeRadians(targetHeading - currH) * 1.2;

            driveMecanum(driveX, driveY, driveR);

            telemetry.addData("Segment", (t < midTime) ? "1" : "2");
            telemetry.update();
        }
        driveMecanum(0,0,0);
    }

    private QuinticCoeffs fitQuintic(double x0, double v0, double a0, double x1, double v1, double a1, double T) {
        QuinticCoeffs q = new QuinticCoeffs();
        q.f = x0; q.e = v0; q.d = 0.5 * a0;
        double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
        q.a = (12*(x0-x1) + 6*(v0+v1)*T + (a0-a1)*T2) / (-2*T5);
        q.b = (30*(x1-x0) - (16*v1+14*v0)*T - (3*a0-2*a1)*T2) / (2*T4);
        q.c = (20*(x0-x1) + (12*v1+8*v0)*T + (3*a0-a1)*T2) / (2*T3);
        return q;
    }

    private void driveMecanum(double x, double y, double r) {
        double pFL = y + x + r; double pBL = y - x + r;
        double pFR = y - x - r; double pBR = y + x - r;
        double max = Math.max(1.0, Math.max(Math.abs(pFL), Math.max(Math.abs(pFR), Math.max(Math.abs(pBL), Math.abs(pBR)))));
        fL.setPower(pFL/max); fR.setPower(pFR/max); bL.setPower(pBL/max); bR.setPower(pBR/max);
    }
}