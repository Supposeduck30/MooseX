package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.apexpathing.util.math.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/**
 * A class that defines the Mecanum Drivetrain: a child class of the Drivetrain
 * @author Sohum Arora 22985
 * @author Krish 26192
 * @author Xander Haemel- 31616
 */
public class MecanumDrive extends Drivetrain {

    private final List<DcMotorEx> motors;
    private final VoltageSensor voltageSensor;
    private final double[] lastMotorPowers;

    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;
    private double nominalVoltage;
    private boolean voltageCompensation;
    private double maxPowerScaling = 1.0;

    private final Vector[] vectors;
    private final Vector[] mecanumVectorsCopy = new Vector[4];
    private final Vector[] truePathingVectors = new Vector[2];

    MecanumConstants constants = new MecanumConstants();

    //default constructor
    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, MecanumConstants mecanumConstants, @NotNull String leftFrontMotorName, @NotNull String rightFrontMotorName, @NotNull String leftRearMotorName, @NotNull String rightRearMotorName) {
        super(hardwareMap, telemetry, mecanumConstants.useBrakeModeInTeleOp, leftFrontMotorName, rightFrontMotorName, leftRearMotorName, rightRearMotorName);

        this.motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        this.lastMotorPowers = new double[]{0, 0, 0, 0};

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.motorCachingThreshold = mecanumConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = mecanumConstants.useBrakeModeInTeleOp;
        this.staticFrictionCoefficient = mecanumConstants.staticFrictionCoefficient;
        this.nominalVoltage = mecanumConstants.nominalVoltage;
        this.voltageCompensation = mecanumConstants.useVoltageCompensation;

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
        breakFollowing();

        Vector copiedFrontLeftVector = Vector.Companion.fromPolar(mecanumConstants.frontLeftVector.magnitude(), mecanumConstants.frontLeftVector.theta());
        vectors = new Vector[]{
                Vector.Companion.fromPolar(copiedFrontLeftVector.magnitude(), copiedFrontLeftVector.theta()),
                Vector.Companion.fromPolar(copiedFrontLeftVector.magnitude(), 2 * Math.PI - copiedFrontLeftVector.theta()),
                Vector.Companion.fromPolar(copiedFrontLeftVector.magnitude(), 2 * Math.PI - copiedFrontLeftVector.theta()),
                Vector.Companion.fromPolar(copiedFrontLeftVector.magnitude(), copiedFrontLeftVector.theta())};

        for (int i = 0; i < 4; i++) {
            mecanumVectorsCopy[i] = new Vector(0, 0);
        }
        truePathingVectors[0] = new Vector(0, 0);
        truePathingVectors[1] = new Vector(0, 0);
    }

    @Override
    public void driveBasedOnInputs(double x, double y, double turn) {
        botCentricDrive(x, y, turn);
    }

    @Override
    public void driveFieldCentric(double x, double y, double turn, double heading) {
        fieldCentricDrive(x, y, turn, heading);
    }

    public void updateConstants() {
        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    private void setMotorsToBrake() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setMotorsToFloat() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void breakFollowing() {
        for (int i = 0; i < motors.size(); i++) {
            lastMotorPowers[i] = 0;
        }
        cutMotorPower();
        setMotorsToFloat();
    }

    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(lastMotorPowers[i] - drivePowers[i]) > motorCachingThreshold ||
                    (drivePowers[i] == 0 && lastMotorPowers[i] != 0)) {
                lastMotorPowers[i] = drivePowers[i];
                setPower(motors.get(i), drivePowers[i]);
            }
        }
    }

    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
    }

    public void fieldCentricDrive(double x, double y, double turn, double robotHeading) {
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double fieldX = x * cos - y * sin;
        double fieldY = x * sin + y * cos;

        fieldX = deadzone(fieldX, 0.05);
        fieldY = deadzone(fieldY, 0.05);
        turn  = deadzone(turn, 0.05);

        double[] powers = new double[]{
                fieldY + fieldX + turn,
                fieldY - fieldX + turn,
                fieldY - fieldX - turn,
                fieldY + fieldX - turn
        };

        double max = Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                Math.max(Math.abs(powers[2]), Math.abs(powers[3])));
        if (max > maxPowerScaling) {
            for (int i = 0; i < 4; i++) powers[i] = (powers[i] / max) * maxPowerScaling;
        }

        runDrive(powers);
    }

    public void botCentricDrive(double x, double y, double turn) {
        double adjX = deadzone(x, 0.05);
        double adjY = deadzone(y, 0.05);
        turn = deadzone(turn, 0.05);

        double[] powers = new double[]{
                adjY + adjX + turn,
                adjY - adjX + turn,
                adjY - adjX - turn,
                adjY + adjX - turn
        };

        double max = Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                Math.max(Math.abs(powers[2]), Math.abs(powers[3])));
        if (max > maxPowerScaling) {
            for (int i = 0; i < 4; i++) powers[i] = (powers[i] / max) * maxPowerScaling;
        }

        runDrive(powers);
    }

    private static double deadzone(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }

    public void getAndRunDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    public double xVelocity() { return constants.xVelocity; }
    public double yVelocity() { return constants.yVelocity; }
    public void setXVelocity(double xMovement) { constants.setXVelocity(xMovement); }
    public void setYVelocity(double yMovement) { constants.setYVelocity(yMovement); }
    public double getStaticFrictionCoefficient() { return staticFrictionCoefficient; }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) /
                (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    public List<DcMotorEx> getMotors() { return motors; }

    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        if (correctivePower.magnitude() > maxPowerScaling)
            correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.magnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.magnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        double[] wheelPowers = new double[4];

        if (correctivePower.magnitude() == maxPowerScaling) {
            truePathingVectors[0].setX(correctivePower.x());
            truePathingVectors[0].setY(correctivePower.y());
            truePathingVectors[1].setX(correctivePower.x());
            truePathingVectors[1].setY(correctivePower.y());
        } else {
            Vector leftSideVector = correctivePower.minus(headingPower);
            Vector rightSideVector = correctivePower.plus(headingPower);

            if (leftSideVector.magnitude() > maxPowerScaling || rightSideVector.magnitude() > maxPowerScaling) {
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower, maxPowerScaling), findNormalizingScaling(correctivePower, headingPower.times(-1.0), maxPowerScaling));
                Vector v0 = correctivePower.minus(headingPower.times(headingScalingFactor));
                Vector v1 = correctivePower.plus(headingPower.times(headingScalingFactor));
                truePathingVectors[0].setX(v0.x());
                truePathingVectors[0].setY(v0.y());
                truePathingVectors[1].setX(v1.x());
                truePathingVectors[1].setY(v1.y());
            } else {
                Vector leftSideVectorWithPathing = leftSideVector.plus(pathingPower);
                Vector rightSideVectorWithPathing = rightSideVector.plus(pathingPower);

                if (leftSideVectorWithPathing.magnitude() > maxPowerScaling || rightSideVectorWithPathing.magnitude() > maxPowerScaling) {
                    double pathingScalingFactor = Math.min(findNormalizingScaling(leftSideVector, pathingPower, maxPowerScaling), findNormalizingScaling(rightSideVector, pathingPower, maxPowerScaling));
                    Vector v0 = leftSideVector.plus(pathingPower.times(pathingScalingFactor));
                    Vector v1 = rightSideVector.plus(pathingPower.times(pathingScalingFactor));
                    truePathingVectors[0].setX(v0.x());
                    truePathingVectors[0].setY(v0.y());
                    truePathingVectors[1].setX(v1.x());
                    truePathingVectors[1].setY(v1.y());
                } else {
                    truePathingVectors[0].setX(leftSideVectorWithPathing.x());
                    truePathingVectors[0].setY(leftSideVectorWithPathing.y());
                    truePathingVectors[1].setX(rightSideVectorWithPathing.x());
                    truePathingVectors[1].setY(rightSideVectorWithPathing.y());
                }
            }
        }

        // truePathingVectors are mutable, times() returns new. Let's fix.
        truePathingVectors[0].setX(truePathingVectors[0].x() * 2.0);
        truePathingVectors[0].setY(truePathingVectors[0].y() * 2.0);
        truePathingVectors[1].setX(truePathingVectors[1].x() * 2.0);
        truePathingVectors[1].setY(truePathingVectors[1].y() * 2.0);

        for (int i = 0; i < 4; i++) {
            mecanumVectorsCopy[i].setX(vectors[i].x());
            mecanumVectorsCopy[i].setY(vectors[i].y());
            mecanumVectorsCopy[i].rotatedVec(robotHeading);
        }

        wheelPowers[0] = (mecanumVectorsCopy[1].x() * truePathingVectors[0].y() - truePathingVectors[0].x() * mecanumVectorsCopy[1].y()) / (mecanumVectorsCopy[1].x() * mecanumVectorsCopy[0].y() - mecanumVectorsCopy[0].x() * mecanumVectorsCopy[1].y());
        wheelPowers[1] = (mecanumVectorsCopy[0].x() * truePathingVectors[0].y() - truePathingVectors[0].x() * mecanumVectorsCopy[0].y()) / (mecanumVectorsCopy[0].x() * mecanumVectorsCopy[1].y() - mecanumVectorsCopy[1].x() * mecanumVectorsCopy[0].y());
        wheelPowers[2] = (mecanumVectorsCopy[3].x() * truePathingVectors[1].y() - truePathingVectors[1].x() * mecanumVectorsCopy[3].y()) / (mecanumVectorsCopy[3].x() * mecanumVectorsCopy[2].y() - mecanumVectorsCopy[2].x() * mecanumVectorsCopy[3].y());
        wheelPowers[3] = (mecanumVectorsCopy[2].x() * truePathingVectors[1].y() - truePathingVectors[1].x() * mecanumVectorsCopy[2].y()) / (mecanumVectorsCopy[2].x() * mecanumVectorsCopy[3].y() - mecanumVectorsCopy[3].x() * mecanumVectorsCopy[2].y());

        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] *= voltageNormalized;
            }
        }

        double max = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])),
                Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));

        if (max > maxPowerScaling) {
            for (int i = 0; i < 4; i++) {
                wheelPowers[i] = (wheelPowers[i] / max) * maxPowerScaling;
            }
        }

        return wheelPowers;
    }

    private double findNormalizingScaling(Vector base, Vector addition, double maxMagnitude) {
        double a = addition.x() * addition.x() + addition.y() * addition.y();
        double b = 2 * (base.x() * addition.x() + base.y() * addition.y());
        double c = base.x() * base.x() + base.y() * base.y() - maxMagnitude * maxMagnitude;
        double discriminant = b * b - 4 * a * c;
        return (-b + Math.sqrt(discriminant)) / (2 * a);
    }
}
