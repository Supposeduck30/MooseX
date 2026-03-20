# Pathing Lib Structure

## Drive System
```mermaid
classDiagram
    class Drivetrain {
        <<interface>>
        +hwManager: HardwareManager
    }
    class HardwareManager {
        - fl, bl, fr, br motors
        - Stuff that u need for directly interacting with the motors
        - Maybe put PID systems for runToVel if needed?
        +setPower methods
    }
    class MecanumDrivetrain {
        impl: Drivetrain
    }

    MecanumDrivetrain <|.. Drivetrain
```
