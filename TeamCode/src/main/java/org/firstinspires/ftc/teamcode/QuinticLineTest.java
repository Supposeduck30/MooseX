package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Quintic Test - Straight Line", group="Drive")
public class QuinticLineTest extends LinearOpMode {

    // --- Hardware ---
    GoBildaPinpointDriver pinpoint;
    DcMotorEx fL, fR, bL, bR;

    // --- Spline Coefficients Holder ---
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
        // 1. Initialize Motors
        fL = hardwareMap.get(DcMotorEx.class, "fl");
        fR = hardwareMap.get(DcMotorEx.class, "fr");
        bL = hardwareMap.get(DcMotorEx.class, "bl");
        bR = hardwareMap.get(DcMotorEx.class, "br");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        // 2. Initialize Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        pinpoint.resetPosAndIMU();

        // 3. Define Trajectory Settings
        // Goal: (0,0) to (24,24) inches.
        // We start/end at 0 velocity, but use "virtual" tangents to force a curve.
        double T = 1.8; // Total time in seconds

        // X-Path: Start 0, End 24.
        // We give a "virtual" start velocity of 30 to "push" the curve forward initially.
        QuinticCoeffs xPath = fitQuintic(0, 30, 0, 24, 0, 0, T);
        // Y-Path: Start 0, End 24.
        // We give a "virtual" end velocity of 30 to "pull" the curve into the finish.
        QuinticCoeffs yPath = fitQuintic(0, 0, 0, 24, 30, 0, T);

        telemetry.addLine("The bot will move from (0,0) to (24, 24)");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive() && getRuntime() < T) {
            double t = getRuntime();
            pinpoint.update();

            // 4. Calculate Targets
            double targetX  = xPath.getPos(t);
            double targetY  = yPath.getPos(t);
            double targetVx = xPath.getVel(t);
            double targetVy = yPath.getVel(t);

            // 5. Calculate Tangential Heading
            double targetHeading = 0;
            if (Math.hypot(targetVx, targetVy) > 0.1) {
                targetHeading = Math.atan2(targetVy, targetVx);
            }

            // 6. Get Current State
            double currX = pinpoint.getPosX(DistanceUnit.INCH);
            double currY = pinpoint.getPosY(DistanceUnit.INCH);
            double currH = pinpoint.getHeading(AngleUnit.RADIANS);

            // 7. PD Control + Feedforward
            // Kp (0.08) handles position error, Kv (0.015) is your "speed" baseline
            double driveX = (targetX - currX) * 0.08 + (targetVx * 0.015);
            double driveY = (targetY - currY) * 0.08 + (targetVy * 0.015);

            // Heading Correction (Normalize to handle the -PI to PI jump)
            double hError = AngleUnit.normalizeRadians(targetHeading - currH);
            double driveR = hError * 1.1;

            // 8. Output to Motors
            driveMecanum(driveX, driveY, driveR);

            telemetry.addData("Time", t);
            telemetry.addData("X-Error", targetX - currX);
            telemetry.addData("Heading", Math.toDegrees(currH));
            telemetry.update();
        }

        driveMecanum(0,0,0);
    }

    // Mathematical Solver for Quintic Polynomial
    private QuinticCoeffs fitQuintic(double x0, double v0, double a0, double x1, double v1, double a1, double T) {
        QuinticCoeffs q = new QuinticCoeffs();
        q.f = x0;
        q.e = v0;
        q.d = 0.5 * a0;
        double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
        q.a = (12*(x0-x1) + 6*(v0+v1)*T + (a0-a1)*T2) / (-2*T5);
        q.b = (30*(x1-x0) - (16*v1+14*v0)*T - (3*a0-2*a1)*T2) / (2*T4);
        q.c = (20*(x0-x1) + (12*v1+8*v0)*T + (3*a0-a1)*T2) / (2*T3);
        return q;
    }

    // Standard Mecanum Kinematics
    private void driveMecanum(double x, double y, double r) {
        double pFL = y + x + r;
        double pBL = y - x + r;
        double pFR = y - x - r;
        double pBR = y + x - r;

        // Clip powers to 1.0 max
        double max = Math.max(1.0, Math.max(Math.abs(pFL), Math.max(Math.abs(pFR),
                     Math.max(Math.abs(pBL), Math.abs(pBR)))));

        fL.setPower(pFL/max);
        fR.setPower(pFR/max);
        bL.setPower(pBL/max);
        bR.setPower(pBR/max);
    }
}