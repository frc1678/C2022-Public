package com.team1678.frc2022;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.lib.util.SwerveModuleConstants;
import com.team1678.frc2022.subsystems.Limelight.LimelightConstants;
import com.team1678.frc2022.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    //CHANGE THIS BASED ON WHAT ROBOT YOU'RE USING. 
    //VALID ENTRIES ARE "ALPHA", "BETA", "COMP", and "EPSILON".
    public static final WantedRobot version = WantedRobot.COMP;

    //Enum toggle for robots
    public enum WantedRobot {
        ALPHA, BETA, COMP, EPSILON
    }

    public enum Current {
        isAlpha, isBeta, isComp, isEpsilon
    }

    public static Current mRobot = Current.isComp;

    public void setCurrent(WantedRobot current) {
        switch (current) {
            case ALPHA:
                mRobot = Current.isAlpha;
                break;
            case BETA:
                mRobot = Current.isBeta;
                break;
            case COMP:
                mRobot = Current.isComp;
                break;
            case EPSILON:
                mRobot = Current.isEpsilon;
                break;
        }
    }
	
	// robot loop time
	public static final double kLooperDt = 0.02;

	/* Control Board */
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;
	
	/* 364 IMPORTED CONSTANTS */
	public static final double stickDeadband = 0.15;

	public static final class SwerveConstants {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelBase = Units.inchesToMeters(20.75);

        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.75;
        public static final double angleGearRatio = 21.43;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Controller Invert */
        public static final boolean invertYAxis = false;
        public static final boolean invertRAxis = false;
        public static final boolean invertXAxis = false; 


        //MODULE SPECIFIC CONSTANTS

        //Front Left Module - Module 0
        public static final class Mod0 {
            public static SwerveModuleConstants SwerveModuleConstants() {
                double wantedOffset = 0;
                switch (mRobot) {
                    case isAlpha:
                        wantedOffset = 334;
                    case isBeta:
                        wantedOffset = 58.71;
                    case isComp:
                        wantedOffset = 123;
                    case isEpsilon:
                        wantedOffset = 420; //Get real value
                }

                return new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER,
                        wantedOffset);
            }
        }
        //Front Right Module - Module 1
        public static final class Mod1 {
            public static SwerveModuleConstants SwerveModuleConstants() {
                double wantedOffset = 0;
                switch (mRobot) {
                    case isAlpha:
                        wantedOffset = 216;
                    case isBeta:
                        wantedOffset = 340.57;
                    case isComp:
                        wantedOffset = 138;
                    case isEpsilon:
                        wantedOffset = 420; //Get real value
                }

                return new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER,
                        wantedOffset);
            }
        }
        //Back Left Module - Module 2
        public static final class Mod2 {
            public static SwerveModuleConstants SwerveModuleConstants() {
                double wantedOffset = 0;
                switch (mRobot) {
                    case isAlpha:
                        wantedOffset = 183;
                    case isBeta:
                        wantedOffset = 343.03;
                    case isComp:
                        wantedOffset = 100;
                    case isEpsilon:
                        wantedOffset = 420; //Get real value
                }

                return new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER,
                        wantedOffset);
            }
        }
        ///Back Right Module - Module 3
        public static final class Mod3 {
            public static SwerveModuleConstants SwerveModuleConstants() {
                double wantedOffset = 0;
                switch (mRobot) {
                    case isAlpha:
                        wantedOffset = 53;
                    case isBeta:
                        wantedOffset = 254.61;
                    case isComp:
                        wantedOffset = 134;
                    case isEpsilon:
                        wantedOffset = 420; //Get real value
                }

                return new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER,
                        wantedOffset);
            }
        }
    }
	
	public static final class SnapConstants {
        public static final double kP = 7.0; // TODO: tune value
        public static final double kI = 0; // TODO: tune value
        public static final double kD = 0.0; // TODO: tune value
        public static final double snapTimeout = 0.25;
        public static final double snapEpsilon = 1.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class VisionAlignConstants {
        public static final double kP = 9.0;
        public static final double kI = 0.001;
        public static final double kD = 0.0;
        public static final double kTimeout = 0.25;
        public static final double kEpsilon = 3.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.2; // TODO: Revise this
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.3; // TODO: Revise this
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0*Math.PI; // TODO: Revise this
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2); // TODO: Revise this

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 5;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        // Trajectory Speed Configs
        public static final TrajectoryConfig defaultSpeedConfig =
                new TrajectoryConfig(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.SwerveConstants.swerveKinematics);
        public static final TrajectoryConfig zeroToDefaultSpeedConfig =
                new TrajectoryConfig(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.SwerveConstants.swerveKinematics)
                        .setStartVelocity(0)
                        .setEndVelocity(kMaxSpeedMetersPerSecond);
        public static final TrajectoryConfig defaultToZeroSpeedConfig =
                new TrajectoryConfig(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.SwerveConstants.swerveKinematics)
                        .setStartVelocity(0)
                        .setEndVelocity(kMaxSpeedMetersPerSecond);
        public static final TrajectoryConfig constantSpeedConfig =
                new TrajectoryConfig(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.SwerveConstants.swerveKinematics)
                        .setStartVelocity(kMaxSpeedMetersPerSecond)
                        .setEndVelocity(kMaxSpeedMetersPerSecond);
    }

    public static final class VisionConstants {
		public static final LimelightConstants kLimelightConstants = new LimelightConstants();
		    static {
                kLimelightConstants.kName = "Limelight";
                kLimelightConstants.kTableName = "limelight";
                kLimelightConstants.kHeight = 0.79; // meters
                kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(34.0);
            }

		public static final double kHorizontalFOV = 59.6; // degrees
		public static final double kVerticalFOV = 49.7; // degrees
		public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
		public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
		public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

        public static final double kCameraFrameRate = 90.0;
        public static final int kDefaultPipeline = 0;
        public static final double kGoalHeight = 2.63; // meters
        public static final double kGoalRadius = 0.678; // meters
	}

    /*** SUBSYSTEM CONSTANTS ***/

    public static final class IntakeConstants {
        public static final double kIntakingVoltage = 10;
        public static final double kSpittingVoltage = -8;
    }

    public static final class ShooterConstants {

        public static final double kFlywheelVelocityConversion = 600.0 / 2048.0; 
        public static final double kAccleratorVelocityConversion = 600.0 / 2048.0 * (1.3 / 1.0);

        public static final double kAcceleratorMultiplier = 0.72;
        
        public static final double kFlywheelTolerance = 500;
        public static final double kShooterP = 0.1; 
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterF = 0.0545;
        public static final double kClosedLoopRamp = 0.1;

        public static final double kAcceleratorP = 0.05;
        public static final double kAcceleratorI = 0.0;
        public static final double kAcceleratorD = 0.0;
        public static final double kAcceleratorF = 0.045;
    }

    public static final class HoodConstants {
        public static final double kCalibratingVoltage = -1.0;
        public static final double kCalibrationCurrentThreshold = 15.0;

        public static final ServoMotorSubsystemConstants kHoodServoConstants = new ServoMotorSubsystemConstants();
        static {
            kHoodServoConstants.kName = "Hood";

            kHoodServoConstants.kMasterConstants.id = Ports.HOOD_ID;
            kHoodServoConstants.kMasterConstants.invert_motor = true;
            kHoodServoConstants.kMasterConstants.invert_sensor_phase = false;

            // Unit == Degrees
            kHoodServoConstants.kHomePosition = 0.0; // Degrees
            kHoodServoConstants.kTicksPerUnitDistance = (2048.0 / 360.0) * (118.4 / 1.0);
            kHoodServoConstants.kKp = 0.70;
            kHoodServoConstants.kKi = 0;
            kHoodServoConstants.kKd = 0;
            kHoodServoConstants.kKf = 0.05;
            kHoodServoConstants.kMaxIntegralAccumulator = 0;
            kHoodServoConstants.kIZone = 0; // Ticks
            kHoodServoConstants.kDeadband = 0; // Ticks

            kHoodServoConstants.kPositionKp = 0.1;
            kHoodServoConstants.kPositionKi = 0;
            kHoodServoConstants.kPositionKd = 0;
            kHoodServoConstants.kPositionKf = 0.0;
            kHoodServoConstants.kPositionMaxIntegralAccumulator = 0;
            kHoodServoConstants.kPositionIZone = 0; // Ticks
            kHoodServoConstants.kPositionDeadband = 0; // Ticks

            kHoodServoConstants.kMinUnitsLimit = 0; // TODO Add actual min/max limits (in degrees)
            kHoodServoConstants.kMaxUnitsLimit = 30;

            kHoodServoConstants.kCruiseVelocity = 20000; // Ticks / 100ms
            kHoodServoConstants.kAcceleration = 20000; // Ticks / 100ms / s
            kHoodServoConstants.kRampRate = 0.0; // s
            kHoodServoConstants.kContinuousCurrentLimit = 35; // amps
            kHoodServoConstants.kPeakCurrentLimit = 40; // amps
            kHoodServoConstants.kPeakCurrentDuration = 10; // milliseconds
            kHoodServoConstants.kMaxVoltage = 3.0;
        }
    }

    public static final class IndexerConstants {

        public static final double kTriggerVelocityConversion = (600.0 / 2048.0) * (1.0 / 2.0);
        public static final double kTriggerP = 0.09; 
        public static final double kTriggerI = 0.0;
        public static final double kTriggerD = 0.0;
        public static final double kTriggerF = 0.046;

        public static final double kTunnelVelocityConversion = (600.0 / 2048.0) * (1.0 / 3.0);
        public static final double kTunnelP = 0.0; 
        public static final double kTunnelI = 0.0;
        public static final double kTunnelD = 0.0;
        public static final double kTunnelF = 0.0545;

        public static final double kSingulatorVoltage = 10.0;
        public static final double kTunnelIndexingVoltage = 5.0;
        public static final double kTunnelReversingVoltage = -5.0;
        public static final double kIdleVoltage = 0.0;
        public static final double kTriggerIndexingVoltage = 4.0;
        public static final double kTriggerReversingVoltage = -5.0;
        public static final double kFeedingVoltage = 3.0;

    }
        
    public static final class ClimberConstants {

    }

    // Timeout constants
    public static final int kLongCANTimeoutMs = 100;
    public static final int kCANTimeoutMs = 10;
}
