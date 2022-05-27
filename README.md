# Team 1678 2022 Robot Code
## Introduction
This repository contains 2022 Rapid React robot code for FRC Team 1678 Citrus Circuits.  

Our robot, Margie, features:
- Field-Relative swerve drive
- Five, Two, and One Ball scoring autonomous modes
- Defensive autonomous modes to obfuscate opponent cargo
- Fully automated color sorting
- Fully automated third Cargo rejection sequence
- Fully automated one-button traverse climb sequence

And more!

## Code Structure
We use an implementation of TimedRobot.  Our robot code is primarily structured around our *Subsystem manager*, which periodically calls methods in each *Subsystem*.  

Our indexing, shooting, and climb logic is contained within our *Superstructure* file.  This file houses the state machine behind the wrong color ejection, third ball rejection, and shooting.  Additionally, our operator commands and auto-climb sequence are also contained here.

Our robot is entirely driven by Falcon 500 motors and their respective Talon FX speed controllers.  We use Thad House's Pi Pico code to read from a Rev V3 color sensor due to concerns with the onboard I2C port.  Our vision is powered by a Limelight 2 Plus.  Other sensors used consist of CANCoders for swerve azimuth, a Pigeon 2 to read yaw and roll, internal sensors within the Falcon 500s, and IR Break Beam sensors within the ball tunnel.

## Notable Files
- [`./Robot.java`](/src/main/java/com/team1678/frc2022/Robot.java)
- [`./subsystems/`](src/main/java/com/team1678/frc2022/subsystems)
	- [`Superstructure.java`](src/main/java/com/team1678/frc2022/subsystems/Superstructure.java) - Indexing, climb sequence, and ejection logic
	- [`Swerve.java`](src/main/java/com/team1678/frc2022/subsystems/Swerve.java) - Swerve drive code including snaps and vision alignment
	- [`Indexer.java`](src/main/java/com/team1678/frc2022/subsystems/Indexer.java) - Slot based indexer system
	- [`ColorSensor.java`](src/main/java/com/team1678/frc2022/subsystems/ColorSensor.java) - Code for parsing data from the color sensor
- [`./auto/`](src/main/java/com/team1678/frc2022/auto)
	- [`modes/`](src/main/java/com/team1678/frc2022/auto/modes) - Contains sequences for all our autonomous modes
	- [`AutoModeSelector.java`](src/main/java/com/team1678/frc2022/auto/AutoModeSelector.java) - Auto mode selection from Shuffleboard
	- [`AutoModeExecutor.java`](src/main/java/com/team1678/frc2022/auto/AutoModeExecutor.java) -  Runs, and stops selected auto mode
- [`./RobotState.java`](src/main/java/com/team1678/frc2022/RobotState.java) - Tracks vehicle to field and goal to field transforms
- [`Standard.vpr`](Standard.vpr) - Our Limelight Pipeline.
## Credits & References Used
- FRC Team 254's Goal Tracking and Pose Estimation
- FRC Team 364's Base Falcon Swerve
- FRC Team 1323's Subsystem Manager and Loopers
- FRC Team 4277's Waypoint Reader
- Thad House's Pico Color Sensor
