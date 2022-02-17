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
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.CrashTracker;
import com.team1678.frc2022.loops.Looper;
import com.team1678.frc2022.subsystems.Climber;
import com.team1678.frc2022.subsystems.Hood;
import com.team1678.frc2022.subsystems.Indexer;
import com.team1678.frc2022.subsystems.Infrastructure;
import com.team1678.frc2022.subsystems.Intake;
import com.team1678.frc2022.subsystems.LEDs;
import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.Shooter;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	private final Hood mHood = Hood.getInstance();
	private final Climber mClimber = Climber.getInstance();
	private final Limelight mLimelight = Limelight.getInstance();
	private final LEDs mLEDs = LEDs.getInstance();

	// logging system
	private LoggingSystem mLogger = LoggingSystem.getInstance();

	private boolean mClimbMode = false;
	private boolean mOpenLoopClimbControlMode = false;
	private boolean mResetClimberPosition = false;
	private boolean mTraversalClimb = false;

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
					mShooter,
					mHood,
					mClimber,
					mSuperstructure,
					mLimelight
			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);
			
			mSubsystemManager.registerLoggingSystems(mLogger);
            mLogger.registerLoops(mLoggingLooper);

			mSwerve.resetOdometry(new Pose2d());
			mSwerve.resetAnglesToAbsolute();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mShuffleBoardInteractions.update();
		mSwerve.outputTelemetry();
		mClimber.outputTelemetry();
	}

	@Override
	public void autonomousInit() {
		CrashTracker.logAutoInit();

		try {

			mEnabledLooper.start();
			mLoggingLooper.start();

			mAutoModeExecutor.start();

			mInfrastructure.setIsDuringAuto(true);
			mLimelight.setPipeline(Constants.VisionConstants.kDefaultPipeline);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	@Override
	public void autonomousPeriodic() {
		mSwerve.updateSwerveOdometry();
		mLimelight.setLed(Limelight.LedMode.ON);
	}

	@Override
	public void teleopInit() {
		try {

			if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

			mSwerve.setModuleStates(
				Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates((
					ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)))));

			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();

			// mInfrastructure.setIsDuringAuto(false);

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
		mLEDs.updateLights();
		try {

			if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

			mLimelight.outputTelemetry();

			// call operator commands container from superstructure
			mSuperstructure.updateOperatorCommands();
			
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

			mSwerve.updateSwerveOdometry();

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
			
			/* CLIMBER */
			if (mControlBoard.getClimbMode()) {
				mClimbMode = true;
			}

			/* Manual Controls */
			if (mClimbMode) {
				if (mControlBoard.getExitClimbMode()) {
					mClimbMode = false;
				}

				if (mControlBoard.operator.getController().getLeftStickButtonPressed()) {
					mOpenLoopClimbControlMode = !mOpenLoopClimbControlMode;
				}

				if (mControlBoard.operator.getController().getRightStickButtonPressed()) {
					mResetClimberPosition = true;
				}

				SmartDashboard.putBoolean("Open Loop Climb", mOpenLoopClimbControlMode);

				if (mResetClimberPosition) {
					mClimber.resetClimberPosition();
					mResetClimberPosition = false;
				}

				if (!mOpenLoopClimbControlMode) {
					if (mControlBoard.operator.getController().getYButtonPressed()) {
						mClimber.setRightClimberPosition(Constants.ClimberConstants.kRightTravelDistance);
					} else if (mControlBoard.operator.getController().getBButtonPressed()) {
						mClimber.setRightClimberPosition(Constants.ClimberConstants.kRightPartialTravelDistance);
					} else if (mControlBoard.operator.getController().getAButtonPressed()){
						mClimber.setRightClimberPosition(0.0);
					}

					int pov = mControlBoard.operator.getController().getPOV();
					if (pov == 0) {
						mClimber.setLeftClimberPosition(Constants.ClimberConstants.kLeftTravelDistance);
					} else if (pov == 270) {
						mClimber.setLeftClimberPosition(Constants.ClimberConstants.kLeftPartialTravelDistance);
					} else if (pov == 180) {
						mClimber.setLeftClimberPosition(0.0);
					}
				} else {
					// set left climber motor
					if (mControlBoard.operator.getController().getPOV() == 0) {
						mClimber.setLeftClimberOpenLoop(8.0);
						//mClimber.setClimberPosition(Constants.ClimberConstants.kClimberHeight);
					} else if (mControlBoard.operator.getController().getPOV() == 180) {
						mClimber.setLeftClimberOpenLoop(-8.0);
						//mClimber.setClimberPosition(Constants.ClimberConstants.kRetractedHeight);
					} else {
						mClimber.setLeftClimberOpenLoop(0.0);
						// mClimber.setLeftClimberPositionDelta(0.0);
					}

					// set right climber motor
					if (mControlBoard.operator.getController().getYButton()) {
						mClimber.setRightClimberOpenLoop(8.0);
						//mClimber.setClimberPosition(Constants.ClimberConstants.kClimberHeight);
					} else if (mControlBoard.operator.getController().getAButton()) {
						mClimber.setRightClimberOpenLoop(-8.0);
						//mClimber.setClimberPosition(Constants.ClimberConstants.kRetractedHeight);
					} else {
						mClimber.setRightClimberOpenLoop(0.0);
						// mClimber.setRightClimberPositionDelta(0.0);
					}
				}
						
				
				/* Automated Climb */ 
				/*
				if (mControlBoard.getTraversalClimb()) {
					mTraversalClimb = true; 
				} else {
					mTraversalClimb = false;
				}

				if (mTraversalClimb) {
					// pull up with the right arm to the mid bar
					mClimber.setRightClimberPosition(0.0);
					mClimber.setLeftClimberPosition(Constants.ClimberConstants.kLeftTravelDistance);

					// pull up with left arm on upper bar while extending right arm to traversal bar
					if (Util.epsilonEquals(mSwerve.getPitch().getDegrees(), // check if dt pitch is at bar contact angle before climbing to next bar
										   Constants.ClimberConstants.kBarContactAngle,
										   Constants.ClimberConstants.kBarContactAngleEpsilon)
						&&
						Util.epsilonEquals(mClimber.getClimberPositionLeft(), // don't climb unless left arm is fully extended
										   Constants.ClimberConstants.kLeftTravelDistance,
										   Constants.ClimberConstants.kTravelDistanceEpsilon)) {

						mClimber.setRightClimberPosition(Constants.ClimberConstants.kRightTravelDistance);
						mClimber.setLeftClimberPosition(0.0);
					}

					// pull up with right arm to partial height on traversal bar
					else if (Util.epsilonEquals(mSwerve.getPitch().getDegrees(), // check if dt pitch is at bar contact angle before climbing to next bar
												Constants.ClimberConstants.kBarContactAngle,
												Constants.ClimberConstants.kBarContactAngleEpsilon)
							&&
							Util.epsilonEquals(mClimber.getClimberPositionRight(), // don't climb unless right arm is fully extended
												Constants.ClimberConstants.kRightTravelDistance,
												Constants.ClimberConstants.kTravelDistanceEpsilon)) {

						mClimber.setRightClimberPosition(Constants.ClimberConstants.kRightPartialTravelDistance);
					}
				} else {
					mClimber.setRightClimberPosition(0.0);
					mClimber.setLeftClimberPosition(0.0);
				}
				*/
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

			mLoggingLooper.stop();

			mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.triggerOutputs();

			mSwerve.setModuleStates(
				Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates((
					ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)))));


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
		mLEDs.updateLights();
		try {

			mAutoModeSelector.updateModeCreator();
			// mSwerve.resetAnglesToAbsolute();

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
