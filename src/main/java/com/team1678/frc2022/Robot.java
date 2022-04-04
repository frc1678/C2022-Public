// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1678.frc2022;

import java.util.Optional;

import com.lib.util.CTREConfigs;
import com.team1678.frc2022.auto.AutoModeExecutor;
import com.team1678.frc2022.auto.AutoModeSelector;
import com.team1678.frc2022.auto.modes.AutoModeBase;
import com.team1678.frc2022.controlboard.ControlBoard;
import com.team1678.frc2022.controlboard.ControlBoard.SwerveCardinal;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.CrashTracker;
import com.team1678.frc2022.loops.Looper;
import com.team1678.frc2022.shuffleboard.ShuffleBoardInteractions;
import com.team1678.frc2022.subsystems.Climber;
import com.team1678.frc2022.subsystems.ColorSensor;
import com.team1678.frc2022.subsystems.Hood;
import com.team1678.frc2022.subsystems.Indexer;
import com.team1678.frc2022.subsystems.Infrastructure;
import com.team1678.frc2022.subsystems.Intake;
import com.team1678.frc2022.subsystems.LEDs;
import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.RobotStateEstimator;
import com.team1678.frc2022.subsystems.Shooter;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;
import com.team1678.frc2022.subsystems.Trigger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.Timer;
import com.team254.lib.wpilib.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */

	 
	// instantiate enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
	// instantiate logging looper
	private final Looper mLoggingLooper = new Looper();

	// declare necessary class objects
	private ShuffleBoardInteractions mShuffleBoardInteractions;
	public static CTREConfigs ctreConfigs;

	// subsystem instances
	private final ControlBoard mControlBoard = ControlBoard.getInstance();

	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final Superstructure mSuperstructure = Superstructure.getInstance();
	private final Swerve mSwerve = Swerve.getInstance();
	private final Infrastructure mInfrastructure = Infrastructure.getInstance();
	private final Intake mIntake = Intake.getInstance();
	private final Indexer mIndexer = Indexer.getInstance();
	private final Shooter mShooter = Shooter.getInstance();
	private final Trigger mTrigger = Trigger.getInstance();
	private final Hood mHood = Hood.getInstance();
	private final ColorSensor mColorSensor = ColorSensor.getInstance();
	private final Climber mClimber = Climber.getInstance();
	private final Limelight mLimelight = Limelight.getInstance();
	private final LEDs mLEDs = LEDs.getInstance();

	// robot state estimator
	private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

	// logging system
	private LoggingSystem mLogger = LoggingSystem.getInstance();

	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit() {
		UsbCamera camera = CameraServer.startAutomaticCapture();
		camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
		MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", 5810);
		cameraServer.setSource(camera);
		cameraServer.setCompression(10);
		
		ctreConfigs = new CTREConfigs();
		mShuffleBoardInteractions = ShuffleBoardInteractions.getInstance();

		try {
			CrashTracker.logRobotInit();

			mSubsystemManager.setSubsystems(			
					mRobotStateEstimator,
					mSwerve,
					mSuperstructure,
					mInfrastructure,
					mIntake,
					mIndexer,
					mShooter,
					mTrigger,
					mHood,
					mColorSensor,
					mClimber,
					mLimelight,
					mLEDs
			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);
			
			mSubsystemManager.registerLoggingSystems(mLogger);
            mLogger.registerLoops(mLoggingLooper);

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());
			mSwerve.resetOdometry(new Pose2d());
			mSwerve.resetAnglesToAbsolute();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mEnabledLooper.outputToSmartDashboard();
		mShuffleBoardInteractions.update();
		mSwerve.outputTelemetry();
		mClimber.outputTelemetry();
	}

	@Override
	public void autonomousInit() {
		CrashTracker.logAutoInit();

		try {
			// reset states
			mSuperstructure.stop();

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mSwerve.resetOdometry(autoMode.get().getStartingPose());
			}

			mAutoModeExecutor.start();
			mLEDs.updateState();

			mInfrastructure.setIsDuringAuto(true);
			mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);
			

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	@Override
	public void autonomousPeriodic() {
		mLimelight.setLed(Limelight.LedMode.ON);
		// mSuperstructure.updateWantEjection();
	}

	@Override
	public void teleopInit() {
		try {

			if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

			// mSwerve.setModuleStates(
			// 	Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates((
			// 		ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)))));

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			// mInfrastructure.setIsDuringAuto(false);

			mSuperstructure.setWantEject(false, false);

			mClimber.setBrakeMode(true);

			mInfrastructure.setIsDuringAuto(false);
		
			mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {

			if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

			mLimelight.setLed(Limelight.LedMode.ON);
			mLimelight.outputTelemetry();

			// call operator commands container from superstructure
			mSuperstructure.updateOperatorCommands();

			mLEDs.updateState();

			
			/* SWERVE DRIVE */
			// hold left bumper
			if (mControlBoard.getBrake()) {
				mSwerve.setLocked(true);
			} else {
				mSwerve.setLocked(false);
			}

			if (mControlBoard.zeroGyro()) {
				mSwerve.zeroGyro();
			}

			if (mControlBoard.getSwerveSnap() != SwerveCardinal.NONE) {
				mSwerve.startSnap(mControlBoard.getSwerveSnap().degrees);
			}
			Translation2d swerveTranslation = new Translation2d(mControlBoard.getSwerveTranslation().x(),
					mControlBoard.getSwerveTranslation().y());
			double swerveRotation = mControlBoard.getSwerveRotation();

			if (mControlBoard.getClimbAlign()) {
				mSwerve.angleAlignDrive(swerveTranslation, 270, true);
			} else if (mControlBoard.getVisionAlign()) {
				mSwerve.visionAlignDrive(swerveTranslation, true);
			} else {
				mSwerve.drive(swerveTranslation, swerveRotation, true, true);
			}

		} catch (Throwable t) {
			t.printStackTrace();
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			// reset states
			mSuperstructure.stop();

			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mDisabledLooper.start();

			mLoggingLooper.stop();

			mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.triggerOutputs();

			// mSwerve.setModuleStates(
			// 	Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates((
			// 		ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)))));


		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		// Reset all auto mode state.
		mAutoModeSelector.reset();
		mAutoModeSelector.updateModeCreator();
		mAutoModeExecutor = new AutoModeExecutor();

	}

	@Override
	public void disabledPeriodic() {
		try {

			mDisabledLooper.outputToSmartDashboard();

			mAutoModeSelector.updateModeCreator();
			
			mSwerve.resetAnglesToAbsolute();

			// update alliance color from driver station while disabled
			mColorSensor.updateAllianceColor();
			mLEDs.updateColor(mColorSensor.getAllianceColor());
			// update baseline color scaling for accurate rb comparison
			// mColorSensor.updateBaselineColorScaling();

			mLimelight.setLed(Limelight.LedMode.ON);
			mLimelight.writePeriodicOutputs();
			mLimelight.outputTelemetry();

			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
				System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			mDisabledLooper.stop();
			mEnabledLooper.stop();

			mLoggingLooper.stop();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}
}
