# Apex Pathing Architecture

## Package Overview

```
com.apexpathing
├── kinematics/      Drivetrain kinematics (swerve, mecanum, tank)
├── follower/        Trajectory generation and following
├── drivetrain/      Drive system implementations
├── localization/    Position tracking (Localizer interface + implementations)
├── hardware/        FTC hardware wrappers (MotorEx, LynxModuleUtil)
└── util/
    ├── math/        Pose, Vector, CoordinateSystem (ApexCoordinates, PedroCoordinates)
    └── ...          Controllers and rate limiters
```

## Drive System

```mermaid
classDiagram
    class Drivetrain {
        <<abstract>>
        + leftFront, leftRear, rightFront, rightRear: DcMotorEx
        + drive(x, y, turn)
        + driveFieldCentric(x, y, turn, heading)
        + stop()
    }
    class MecanumDrive {
        - constants: MecanumConstants
        + botCentricDrive(x, y, turn)
        + fieldCentricDrive(x, y, turn, heading)
        + calculateDrive(corrective, heading, pathing, robotHeading)
    }
    class TankDrive {
        - fourWheelDrive: boolean
    }
    class CustomDrive {
        <<abstract>>
        # localizer: Localizer
        # controller: HolonomicTrajectoryFollower
        + setDrivePowers(Pose)
        + update()
        + followTrajectory(Trajectory)
    }
    class SwerveDrive {
        - modules: SwerveModule[4]
        - kinematics: SwerveKinematics
    }
    class SwerveModule {
        - driveMotor: MotorEx
        - turnServo: Servo
        - absoluteEncoder: AnalogInput
        + setDesiredState(velocity, angle)
    }

    Drivetrain <|-- MecanumDrive
    Drivetrain <|-- TankDrive
    CustomDrive <|-- SwerveDrive
    SwerveDrive *-- SwerveModule
    MecanumDrive --> MecanumConstants
```

## Kinematics

```mermaid
classDiagram
    class KinematicsSwitcher {
        + setDriveType(Kinematics)
        + get(): Kinematics
    }
    class Kinematics {
        <<interface>>
        + toWheelSpeeds(ChassisSpeeds): Object
    }

    Kinematics <|-- SwerveKinematics
    Kinematics <|-- MecanumKinematics
    Kinematics <|-- TankKinematics
```

## Localization

```mermaid
classDiagram
    class Localizer {
        <<interface>>
        + update()
        + getPose(): Pose
        + getVelocity(): Pose
        + setPose(Pose)
    }
    class PinpointLocalizer {
        - pinpoint: GoBildaPinpointDriver
        + init()
    }

    Localizer <|.. PinpointLocalizer
```

## Math Utilities

```mermaid
classDiagram
    class Pose {
        + x(), y(), heading(): Double
        + coordSystem: CoordinateSystem
        + distanceTo(Pose): Double
        + inCoordinateSystem(CoordinateSystem): Pose
        + rotate(theta): Pose
        + rotated(theta): Pose
        + reflectX(at), reflectY(at)
        + asVector(): Vector
    }
    class CoordinateSystem {
        <<interface>>
        + toApexCoordinates(Pose): Pose
        + fromApexCoordinates(Pose): Pose
    }
    class ApexCoordinates {
        <<object>>
    }
    class PedroCoordinates {
        <<object>>
    }

    CoordinateSystem <|.. ApexCoordinates
    CoordinateSystem <|.. PedroCoordinates
    Pose --> CoordinateSystem
```

## Trajectory Following

```mermaid
classDiagram
    class HolonomicTrajectoryFollower {
        + update(currentPose, target): Object
    }
    class QuinticHermiteSpline {
        + getPoint(t): Vector
        + getVelocity(t): Vector
        + getAcceleration(t): Vector
    }
    class ArcLengthParameterizer {
        + getT(s): double
        + getTotalArcLength(): double
    }

    HolonomicTrajectoryFollower --> TrajectorySample
    ArcLengthParameterizer --> QuinticHermiteSpline
```
