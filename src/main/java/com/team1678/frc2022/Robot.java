// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1678.frc2022;

import java.util.Optional;

import com.team1678.frc2022.auto.AutoModeExecutor;
import com.team1678.frc2022.auto.AutoModeSelector;
import com.team1678.frc2022.auto.modes.AutoModeBase;
import com.team1678.frc2022.controlboard.ControlBoard;
import com.team1678.frc2022.controlboard.ControlBoard.SwerveCardinal;
import com.team1678.frc2022.loops.CrashTracker;
import com.team1678.frc2022.loops.Looper;
import com.team1678.frc2022.subsystems.Indexer;
import com.team1678.frc2022.subsystems.Infrastructure;
import com.team1678.frc2022.subsystems.Intake;
import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.RobotStateEstimator;
import com.team1678.frc2022.subsystems.Shooter;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

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

	/* Declare necessary class objects */
	private ShuffleBoardInteractions mShuffleBoardInteractions;
	public static CTREConfigs ctreConfigs;

	// subsystem instances
	private final ControlBoard mControlBoard = ControlBoard.getInstance();

	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final Swerve mSwerve = Swerve.getInstance();
	private final Infrastructure mInfrastructure = Infrastructure.getInstance();
	private final Intake mIntake = Intake.getInstance();
	private final Indexer mIndexer = Indexer.getInstance();
	private final Shooter mShooter = Shooter.getInstance();
	private final Superstructure mSuperstructure = Superstructure.getInstance();
	private final Limelight mLimelight = Limelight.getInstance();

	private final RobotState mRobotState = RobotState.getInstance();
	private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

	// instantiate enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();

	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit() {
		ctreConfigs = new CTREConfigs();
		mShuffleBoardInteractions = ShuffleBoardInteractions.getInstance();

		try {
			CrashTracker.logRobotInit();

			mSubsystemManager.setSubsystems(
					mRobotStateEstimator,
					mSwerve,
					mInfrastructure,
					mIntake,
					mIndexer,
					mShooter,
					mSuperstructure,
					mLimelight
			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			mSwerve.resetOdometry(new Pose2d());
			// Robot starts forwards.
			mRobotState.reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mShuffleBoardInteractions.update();
		// mRobotState.outputToSmartDashboard();
	}

	@Override
	public void autonomousInit() {
		CrashTracker.logAutoInit();

		try {

			mEnabledLooper.start();
			mAutoModeExecutor.start();

			mInfrastructure.setIsDuringAuto(true);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		try {

			mDisabledLooper.stop();
			mEnabledLooper.start();

			mInfrastructure.setIsDuringAuto(false);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {
			/* SWERVE DRIVE */
			if (mControlBoard.zeroGyro()) {
				mSwerve.zeroGyro();
			}

			if (mControlBoard.getSwerveSnap() != SwerveCardinal.NONE) {
				mSwerve.startSnap(mControlBoard.getSwerveSnap().degrees);
			}
			Translation2d swerveTranslation = new Translation2d(mControlBoard.getSwerveTranslation().x(),
					mControlBoard.getSwerveTranslation().y());
			double swerveRotation = mControlBoard.getSwerveRotation();

			if (mControlBoard.getVisionAlign()) {
				mSwerve.visionAlignDrive(swerveTranslation, true, true);
			} else {
				mSwerve.drive(swerveTranslation, swerveRotation, true, true);
			}

			// Intake
			if (mControlBoard.getIntake()) {
				mIntake.setState(Intake.WantedAction.INTAKE);
			} else if (mControlBoard.getOuttake()) {
				mIntake.setState(Intake.WantedAction.REVERSE);
			} else if (mControlBoard.getSpitting()) {
				mIntake.setState(Intake.WantedAction.SPIT);
			} else {
				mIntake.setState(Intake.WantedAction.NONE);
			}

			if (mControlBoard.operator.getController().getYButtonPressed()) {
				mSuperstructure.setWantShoot();
			}

			if (mControlBoard.operator.getController().getAButtonPressed()) {
				mSuperstructure.setShooterVelocity(1800);
				mSuperstructure.setWantSpinUp();
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {

			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mDisabledLooper.start();

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), new com.team254.lib.geometry.Pose2d());

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

			mAutoModeSelector.updateModeCreator();
			// [mSwerve.resetAnglesToAbsolute();

			mLimelight.setLed(Limelight.LedMode.ON);
			mLimelight.writePeriodicOutputs();

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
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}
}
