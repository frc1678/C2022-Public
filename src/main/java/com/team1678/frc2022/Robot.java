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
import com.team1678.frc2022.subsystems.Shooter;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

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
	private final Superstructure mSuperstructure = Superstructure.getInstance();
	// private final Shooter mShooter = Shooter.getInstance();
	private final Swerve mSwerve = Swerve.getInstance();
	private final Intake mIntake = Intake.getInstance();
	// private final Limelight mLimelight = Limelight.getInstance();
	private final Infrastructure mInfrastructure = Infrastructure.getInstance();
	private final Indexer mIndexer = Indexer.getInstance();

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
					mSwerve,
					mInfrastructure,
					mIntake,
					mIndexer,
					// mShooter,
					mSuperstructure // ,
					// mLimelight
			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			mSwerve.resetOdometry(new Pose2d());
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mShuffleBoardInteractions.update();
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
		mSwerve.updateSwerveOdometry();
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

			mSwerve.updateSwerveOdometry();

			if (mControlBoard.getSwerveSnap() != SwerveCardinal.NONE) {
				mSwerve.startSnap(mControlBoard.getSwerveSnap().degrees);
			}
			Translation2d swerveTranslation = new Translation2d(mControlBoard.getSwerveTranslation().x(),
					mControlBoard.getSwerveTranslation().y());
			double swerveRotation = mControlBoard.getSwerveRotation();

			// if (mControlBoard.getVisionAlign()) {
			// 	mSwerve.visionAlignDrive(swerveTranslation, true, true);
			// } else {
				mSwerve.drive(swerveTranslation, swerveRotation, true, true);
			// }

			// Intake
			if (mControlBoard.getIntake()) {
				mSuperstructure.setWantIntake(true);
			} else if (mControlBoard.getOuttake()) {
				mSuperstructure.setWantOuttake(true);
			} else {
				mSuperstructure.setWantIntake(false);
				mSuperstructure.setWantOuttake(false);
			}

			// if (mControlBoard.operator.getController().getYButtonPressed()) {
			// 	mSuperstructure.setWantShoot();
			// }

			// if (mControlBoard.operator.getController().getAButtonPressed()) {
			// 	mSuperstructure.setShooterVelocity(1800);
			// 	mSuperstructure.setWantSpinUp();
			// }

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

			// mLimelight.setLed(Limelight.LedMode.ON);
			// mLimelight.writePeriodicOutputs();

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
