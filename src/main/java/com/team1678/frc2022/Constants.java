package com.team1678.frc2022;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.lib.util.SwerveModuleConstants;
import com.team1678.frc2022.subsystems.Limelight.LimelightConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
	
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
        public static final double trackWidth = Units.inchesToMeters(29.5); // TODO: Check value
		public static final double wheelBase = Units.inchesToMeters(29.5); // TODO: Check value

        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.86; // TODO: Check value
        public static final double angleGearRatio = 12.8; // TODO: Check value

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
        public static final double angleKP = 0.3; // TODO: Check value
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // TODO: Check value
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
        public static final boolean driveMotorInvert = false; // TODO: Check value
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Controller Invert */
        public static final boolean invertXAxis = false; // TODO: Check value 
        public static final boolean invertYAxis = false; // TODO: Check value
        public static final boolean invertRAxis = false; // TODO: Check value


        /*** MODULE SPECIFIC CONSTANTS ***/
        
		/* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final double angleOffset = 144;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final double angleOffset = 44;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(Ports.FR_DRIVE, Ports.FR_ROTATION, Ports.FR_CANCODER, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final double angleOffset = 289;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(Ports.BL_DRIVE, Ports.BL_ROTATION, Ports.BL_CANCODER, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final double angleOffset = 60;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(Ports.BR_DRIVE, Ports.BR_ROTATION, Ports.BR_CANCODER, angleOffset);
        }

	}
	
	public static final class SnapConstants {
        public static final double snapKP = 3.0;
        public static final double snapKI = 0;
        public static final double snapKD = 0.0;
        public static final double snapTimeout = 0.25;
        public static final double snapEpsilon = 1.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.5; // TODO: Revise this
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // TODO: Revise this
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0*Math.PI; // TODO: Revise this
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2); // TODO: Revise this
		
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 2.5;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
				
		// Trajectory Speed Configs
		public static final TrajectoryConfig defaultConfig = 
		new TrajectoryConfig(
				Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.SwerveConstants.swerveKinematics);

	  }

	  public static final class VisionConstants {
		public static final LimelightConstants kLimelightConstants = new LimelightConstants();
		static {
			kLimelightConstants.kName = "Limelight";
			kLimelightConstants.kTableName = "limelight";
			kLimelightConstants.kHeight = 24.5; // inches
			kLimelightConstants.kTurretToLens = Pose2d.identity();
			kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(0.0);
		}

		public static final double kHorizontalFOV = 59.6; // degrees
		public static final double kVerticalFOV = 49.7; // degrees
		public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
		public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
		public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
	}

    /*** SUBSYSTEM CONSTANTS ***/
    public static final class IntakeConstants {
    
    }

    public static final class HopperConstants {
        
    }

    public static final class ElevatorConstants {
        
    }

    public static final class ShooterConstants {
        
    }

    public static final class ClimberConstants {
        
    }

	// Timeout constants
	public static final int kLongCANTimeoutMs = 100;
	public static final int kCANTimeoutMs = 10;
}
