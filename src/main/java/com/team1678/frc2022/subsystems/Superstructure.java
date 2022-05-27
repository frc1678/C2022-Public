package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.RobotState;
import com.team1678.frc2022.controlboard.ControlBoard;
import com.team1678.frc2022.drivers.Pigeon;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.regressions.ShooterRegression;
import com.team1678.frc2022.subsystems.LEDs.State;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.InterpolatingDouble;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Optional;

public class Superstructure extends Subsystem {

    // superstructure instance
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    /*** REQUIRED INSTANCES ***/
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Swerve mSwerve = Swerve.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final ColorSensor mColorSensor = ColorSensor.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Trigger mTrigger = Trigger.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Climber mClimber = Climber.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();
    private final LEDs mLEDs = LEDs.getInstance();
    private final Pigeon mPigeon = Pigeon.getInstance();

    // robot state
    private final RobotState mRobotState = RobotState.getInstance();

    // timer for reversing the intake and then stopping it once we have two correct
    // cargo
    Timer mIntakeRejectTimer = new Timer();
    // timer for asserting ball position
    Timer mAssertBallPositionTimer = new Timer();

    // PeriodicIO instance and paired csv writer
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    /*** CONTAINER FOR SUPERSTRUCTURE ACTIONS AND GOALS ***/
    public static class PeriodicIO {
        // INPUTS
        // (superstructure actions)
        private boolean INTAKE = false; // run the intake to pick up cargo
        private boolean REVERSE = false; // reverse the intake and singulator
        private boolean REJECT = false; // have the intake reject cargo
        private boolean PREP = false; // spin up and aim with shooting setpoints
        private boolean SHOOT = false; // shoot cargo
        private boolean FENDER = false; // shoot cargo from up against the hub
        private boolean SPIT = false; // spit cargo from shooter at low velocity

        // time measurements
        public double timestamp;
        public double dt;

        // OUTPUTS
        // (superstructure goals/setpoints)
        private Intake.WantedAction real_intake = Intake.WantedAction.NONE;
        private Trigger.WantedAction real_trigger = Trigger.WantedAction.NONE;
        private double real_shooter = 0.0;
        private double real_hood = 0.0;
    }

    /* Setpoint Tracker Variables */
    public int mBallCount = 0; // number of balls in robot

    // shooting system setpoints
    public double mShooterSetpoint = 1000.0;
    public double mHoodSetpoint = 20.0;
    private double mHoodAngleAdjustment = 0.0; // on the fly shot angle adjustments
    private boolean mResetHoodAngleAdjustment = false; // reset hood angle adjustment

    // intake / eject locking status
    private boolean mIntakeReject = false; // reject third ball from entering intake
    private boolean mIntakeOverride = false; // backup override for intake locking logic
    private boolean mForceEject = false; // manually run the ejector
    private boolean mDisableEjecting = false; // backup override for ejecting logic
    private boolean mSlowEject = false; // eject at lower velocity for aimed ejection during auto

    // climb mode tracker variables
    private boolean mClimbMode = false; // if we are in climb mode (lock out other functions when climbing)
    private boolean mOpenLoopClimbControlMode = false; // open loop climber control for manual jogging
    private boolean mResetClimberPosition = false; // re-zeroing climber arms
    private boolean mAutoTraversalClimb = false; // if we are running auto-traverse
    private boolean mAutoHighBarClimb = false; // if we are running auto-high
    private int mClimbStep = 0; // step of auto-climb we are currently on
    private double mRoll = 0.0; // roll of the robot

    // fender shot constants
    private final double kFenderVelocity = 2200;
    private final double kFenderAngle = 14.0;

    // spit shot constants
    private final double kSpitVelocity = 1000;
    private final double kSpitAngle = 20.0;

    // aiming parameter vars
    private Optional<AimingParameters> real_aiming_params_ = Optional.empty();
    private int mTrackId = -1;
    private double mTargetAngle = 0.0;
    private double mCorrectedDistanceToTarget = 0.0;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();

                if (!mClimbMode) {
                    updateBallCounter();
                    updateSpitState();
                    updateVisionAimingParameters();
                    updateShootingSetpoints();
                }

                setGoals();
                updateRumble();

                // send log data
                SendLog();

                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    /*** SETTERS FOR SUPERSTRUCTURE ACTIONS OUTSIDE OPERATOR INPUT ***/
    public void setWantIntake(boolean intake) {
        mPeriodicIO.INTAKE = intake;

        // set other intake actions to false if this action is true
        if (mPeriodicIO.INTAKE) {
            mPeriodicIO.REVERSE = false;
            mPeriodicIO.REJECT = false;
        }
    }

    public void setWantReverse(boolean reverse) {
        mPeriodicIO.REVERSE = reverse;

        // set other intake actions to false if this action is true
        if (mPeriodicIO.REVERSE) {
            mPeriodicIO.INTAKE = false;
            mPeriodicIO.REJECT = false;
        }
    }

    public void setWantReject(boolean reject) {
        mPeriodicIO.REJECT = reject;

        // set other intake actions to false if this action is true
        if (mPeriodicIO.REJECT) {
            mPeriodicIO.INTAKE = false;
            mPeriodicIO.REVERSE = false;
        }
    }

    public void setWantIntakeNone() {
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.REVERSE = false;
        mPeriodicIO.REJECT = false;
    }

    public void setWantEject(boolean eject, boolean slow_eject) {
        mForceEject = eject;
        mSlowEject = slow_eject;
    }

    public void setSlowEject(boolean slow_eject) {
        mSlowEject = slow_eject;
    }

    public void setEjectDisable(boolean enable) {
        mDisableEjecting = enable;
    }

    public void setWantPrep(boolean wants_prep) {
        mPeriodicIO.PREP = wants_prep;
    }

    public void setWantShoot(boolean wants_shoot) {
        mPeriodicIO.SHOOT = wants_shoot;
    }

    public void setShootingParameters(double flywheel, double hood) {
        mShooterSetpoint = flywheel;
        mHoodSetpoint = hood;
    }

    /***
     * CONTAINER FOR OPERATOR COMMANDS CALLING SUPERSTRUCTURE ACTIONS
     * 
     * Intaking
     * - hold right trigger to intake
     * - hold left trigger to manual eject
     * 
     * Shooting
     * - press A to prep for shot (spin up)
     * - press Y to shoot once ready
     * - press B to toggle fender shot with set params
     * - press X to toggle spit shot with set params
     * 
     * Manual Hood Adjustment
     * - Use dpad to manually adjust hood with offset
     * --> 0 to move hood up
     * --> 180 to move hood down
     * - press START button to reset adjustment
     * 
     * Other Manual Sets
     * - press dpad left (POV 270) to toggle force intake
     * - press dpad right (POV 90) to toggle disabling the ejector
     * - hold left bumper to eject balls manuallyo
     * 
     * Climb Controls
     * - press left bumper, right bumper, left trigger, right trigger to enter climb
     * mode
     * - press center "back" and "start" buttons to exit climb mode
     * - press down on left joystick to toggle open loop control of climber arms
     * - press down on right joystick to zero position on climber motors
     * 
     * - press A to prep for climb and extend right arm
     * - press B to only climb to mid bar
     * - press Y to complete full automated climb to traversal bar
     * 
     * - DPad down (POV 180) to climb mid bar and extend for high bar
     * - DPad right (POV 90) to climb high bar and extend for traversal bar
     * - DPad up (POV 0) to climb traversal bar
     * 
     */
    public void updateOperatorCommands() {

        // reset gyro yaw when not climbing to eliminate drift
        if (mClimbStep == 0) {
            mPigeon.setRoll(0.0);
        }

        // reset climb tracker variables when entering climb mode
        if (mControlBoard.getEnterClimbMode()) {
            mClimbMode = true;
            mClimbStep = 0;
            mOpenLoopClimbControlMode = false;
            mForceEject = false;
        }

        if (mClimbMode) {
            /*** CLIMB MODE CONTROLS ***/

            // stop all other superstructure actions
            stop();

            if (mControlBoard.getExitClimbMode()) {
                mClimbMode = false;
                mAutoTraversalClimb = false;
                mAutoHighBarClimb = false;
            }

            if (mControlBoard.getToggleOpenLoopClimbMode()) {
                mOpenLoopClimbControlMode = !mOpenLoopClimbControlMode;
                mClimber.setClimberNone();
                mAutoTraversalClimb = false;
                mAutoHighBarClimb = false;
            }

            if (mControlBoard.getClimberRezero()) {
                mResetClimberPosition = true;
            }

            if (mResetClimberPosition) {
                mClimber.resetClimberPosition();
                mClimber.setClimberNone();
                mResetClimberPosition = false;
                mClimbStep = 0;
            }

            if (!mOpenLoopClimbControlMode) {
                if (mControlBoard.getClimberRetract()) {
                    mClimber.setClimberNone();
                    mClimbStep = 0;
                    mAutoTraversalClimb = false;
                    mAutoHighBarClimb = false;

                } else if (mControlBoard.getClimberExtend()) {
                    // climb step 1
                    mClimber.setExtendForClimb();
                    mAutoTraversalClimb = false;
                    mAutoHighBarClimb = false;
                    mClimbStep = 1;

                } else if (mControlBoard.getTraversalClimb()) {
                    mAutoTraversalClimb = true;
                    mAutoHighBarClimb = false;

                } else if (mControlBoard.getHighBarClimb()) {
                    mAutoHighBarClimb = true;
                    mAutoTraversalClimb = false;

                } else {
                    // backup manual climb controls
                    if (mControlBoard.operator.getController().getBButtonPressed()) {
                        mClimber.setClimbMidBarAndExtend();

                    } else if (mControlBoard.operator.getController().getPOV() == 180) {
                        mClimber.setHighBarExtend();

                    } else if (mControlBoard.operator.getController().getPOV() == 90) {
                        mClimber.setClimbHighBarAndExtend();

                    } else if (mControlBoard.operator.getController().getPOV() == 0) {
                        mClimber.setTraversalBarExtend();

                    } else if (mControlBoard.operator.getController().getPOV() == 270) {
                        mClimber.setClimbTraversalBar();
                    }
                }

                if (mAutoTraversalClimb) {

                    // climb mid bar and extend to high bar
                    if (mClimbStep == 1) {
                        mClimber.setClimbMidBarAndExtend();
                        mClimbStep++; // climb step 2
                    }

                    // set left arm to full extension from partial height to make contact on high
                    // bar
                    if (mRoll < Constants.ClimberConstants.kHighBarExtendAngle // check if dt roll is past high bar
                                                                               // while swinging to extend
                            &&
                            Util.epsilonEquals(mClimber.getClimberPositionLeft(), // don't extend unless left arm is at
                                                                                  // partial height
                                    Constants.ClimberConstants.kLeftPartialTravelDistance,
                                    Constants.ClimberConstants.kTravelDistanceEpsilon)
                            && (mClimbStep == 2)) {

                        mClimber.setHighBarExtend();
                        mClimbStep++; // climb step 3
                    }

                    // pull up with left arm on upper bar while extending right arm to traversal bar
                    if ((mRoll > Constants.ClimberConstants.kHighBarContactAngle) // check if dt roll is at bar contact
                                                                                  // angle before climbing to next bar
                            &&
                            Util.epsilonEquals(mClimber.getClimberPositionLeft(), // don't climb unless left arm is
                                                                                  // fully extended
                                    Constants.ClimberConstants.kLeftTravelDistance,
                                    Constants.ClimberConstants.kTravelDistanceEpsilon)
                            && (mClimbStep == 3)) {

                        mClimber.setClimbHighBarAndExtend();
                        mClimbStep++; // climb step 4
                    }

                    // set right arm to full extension from partial height to make contact on
                    // traversal bar
                    if (mRoll > Constants.ClimberConstants.kTraversalBarExtendAngle // check if dt roll is past
                                                                                    // traversal bar while swinging to
                                                                                    // extend
                            &&
                            Util.epsilonEquals(mClimber.getClimberPositionRight(), // don't extend unless right arm is
                                                                                   // at partial height
                                    Constants.ClimberConstants.kRightPartialTravelDistance,
                                    Constants.ClimberConstants.kTravelDistanceEpsilon)
                            && (mClimbStep == 4)) {

                        mClimber.setTraversalBarExtend();
                        mClimbStep++; // climb step 5
                    }

                    // climb on the right arm after we are fully extended on traversal bar
                    if (Util.epsilonEquals(mRoll, // check if dt roll is at the angle necessary
                            Constants.ClimberConstants.kTraversalBarContactAngle,
                            2.0)
                            &&
                            Util.epsilonEquals(mClimber.getClimberPositionRight(), // don't extend unless right arm is
                                                                                   // at full height
                                    Constants.ClimberConstants.kRightTravelDistance,
                                    Constants.ClimberConstants.kTravelDistanceEpsilon)
                            && (mClimbStep == 5)) {

                        mClimber.setClimbTraversalBar();
                        mClimbStep++; // climb step 6
                    }

                } else if (mAutoHighBarClimb) {

                    // climb mid bar and extend to high bar
                    if (mClimbStep == 1) {
                        mClimber.setClimbMidBarAndExtend();
                        mClimbStep++; // climb step 2
                    }

                    // set left arm to full extension from partial height to make contact on high
                    // bar
                    if (mRoll < Constants.ClimberConstants.kHighBarExtendAngle // check if dt roll is past high bar
                                                                               // while swinging to extend
                            &&
                            Util.epsilonEquals(mClimber.getClimberPositionLeft(), // don't extend unless left arm is at
                                                                                  // partial height
                                    Constants.ClimberConstants.kLeftPartialTravelDistance,
                                    Constants.ClimberConstants.kTravelDistanceEpsilon)
                            && (mClimbStep == 2)) {

                        mClimber.setHighBarExtend();
                        mClimbStep++; // climb step 3
                    }

                    // pull up partially with left arm on high bar and end
                    if ((mRoll > Constants.ClimberConstants.kHighBarContactAngle) // check if dt roll is at bar contact
                                                                                  // angle before climbing to next bar
                            &&
                            Util.epsilonEquals(mClimber.getClimberPositionLeft(), // don't climb unless left arm is
                                                                                  // fully extended
                                    Constants.ClimberConstants.kLeftTravelDistance,
                                    Constants.ClimberConstants.kTravelDistanceEpsilon)
                            && (mClimbStep == 3)) {

                        mClimber.setHighBarPartialClimb();
                        mClimbStep++; // climb step 4
                    }
                }

            } else {

                // open loop climb jog
                if (mControlBoard.operator.getController().getPOV() == 0) {
                    mClimber.setLeftClimberOpenLoop(8.0);
                } else if (mControlBoard.operator.getController().getPOV() == 180) {
                    mClimber.setLeftClimberOpenLoop(-8.0);
                } else {
                    mClimber.setLeftClimberOpenLoop(0.0);
                }

                if (mControlBoard.operator.getController().getYButton()) {
                    mClimber.setRightClimberOpenLoop(8.0);
                } else if (mControlBoard.operator.getController().getAButton()) {
                    mClimber.setRightClimberOpenLoop(-8.0);
                } else {
                    mClimber.setRightClimberOpenLoop(0.0);
                }

            }

        } else {

            /*** NORMAL TELEOP CONTROLS ***/

            // disable toggle for intake locking
            if (mControlBoard.getDisableIntakeLogic()) {
                mIntakeOverride = !mIntakeOverride;
            }

            if (mControlBoard.getIntake()) {

                // lock intake control and start a rejection sequence if when intaking a third ball
                if ((indexerFull() || stopIntaking()) && !mIntakeReject) {
                    mIntakeReject = true;
                }

                // reverse intake for X seconds to ensure third ball has left system
                if (mIntakeReject && !mIntakeOverride) {
                    setWantReject(true);
                    if (!(mIndexer.getTopBeamBreak() && mColorSensor.getForwardBeamBreak())
                            && !indexerFull()
                            && !stopIntaking()) {

                        mIntakeReject = false;
                    }
                } else { // if we aren't rejecting
                    setWantIntake(true);
                }
            } else {
                mIntakeRejectTimer.reset();
                if (mControlBoard.getReject()) {
                    setWantReverse(true);
                } else {
                    setWantIntakeNone();
                }
            }

            // toggle ejecting to disable if necessary
            if (mControlBoard.getDisableColorLogic()) {
                mDisableEjecting = !mDisableEjecting;
            }

            // if we want to manual eject
            mForceEject = mControlBoard.getManualEject();

            // control shooting
            if (mControlBoard.getShoot()) {
                mPeriodicIO.SHOOT = !mPeriodicIO.SHOOT;

                // reset intake actions
                setWantIntakeNone();
            }

            // spin up to shoot if we aren't already
            if ((mPeriodicIO.SHOOT || mPeriodicIO.SPIT) && !mPeriodicIO.PREP) {
                mPeriodicIO.PREP = true;
            }

            // control prep
            if (mControlBoard.getPrep()) {
                mPeriodicIO.PREP = !mPeriodicIO.PREP;
            }

            // control fender shot
            if (mControlBoard.getFender()) {
                mPeriodicIO.FENDER = !mPeriodicIO.FENDER;
            }

            // non-toggle one ball spit shot
            if (mControlBoard.getSpit()) {
                mPeriodicIO.SPIT = true;
            }

            // control for adding manual hood adjustment
            switch (mControlBoard.getHoodManualAdjustment()) {
                case 1:
                    mHoodAngleAdjustment += 1;
                    break;
                case -1:
                    mHoodAngleAdjustment += -1;
                    break;
                case 0:
                    mHoodAngleAdjustment += 0;
                    break;
            }
            // reset manual hood adjustment if necessary
            if (mControlBoard.getResetHoodAdjust()) {
                mResetHoodAngleAdjustment = true;
            }
        }
    }

    /*** UPDATE BALL COUNTER FOR INDEXING STATUS ***/
    public void updateBallCounter() {
        if (mIndexer.hasTopBall() && mIndexer.hasBottomBall()) {
            mBallCount = 2;
        } else if (mIndexer.hasTopBall() || mIndexer.hasBottomBall()) {
            mBallCount = 1;
        } else {
            mBallCount = 0;
        }
    }

    /***
     * UPDATE SPIT STATE 
     * Controls sequence to spit out one ball at a time
     **/
    public void updateSpitState() {
        // when two balls are in the indexer:
        if (mBallCount == 2) {
            // check if ball count decreases
            // additional case for if we see two balls pass beam break continuous
            if ((mBallCount < 2) || (!mIndexer.getBottomBeamBreak() || !mIndexer.getTopBeamBreak())) {
                mPeriodicIO.SPIT = false;
            }
        }
        if (mBallCount == 1) {
            mAssertBallPositionTimer.start();

            // check if ball count decreases
            if ((mBallCount == 0)
                    && mAssertBallPositionTimer.hasElapsed(Constants.IndexerConstants.kBallAssertionTime)) {
                mPeriodicIO.SPIT = false;

                mAssertBallPositionTimer.stop();
                mAssertBallPositionTimer.reset();
            }
        }
        // if we don't have any balls we should never be in a spitting state
        if (mBallCount == 0) {
            mPeriodicIO.SPIT = false;
        }
    }

    /*** RUMBLE OPERATOR CONTROLLERS WHILE SHOOTING ***/
    public void updateRumble() {
        if (!mClimbMode) {
            mControlBoard.setOperatorRumble(mPeriodicIO.SHOOT);
            mControlBoard.setDriverRumble(mPeriodicIO.SHOOT);
        } else {
            mControlBoard.setOperatorRumble(false);
            mControlBoard.setDriverRumble(false);
        }
    }

    /***
     * GET REAL AIMING PARAMETERS
     * called in updateVisionAimingSetpoints()
     */
    public Optional<AimingParameters> getRealAimingParameters() {
        Optional<AimingParameters> aiming_params = RobotState.getInstance().getAimingParameters(mTrackId,
                Constants.VisionConstants.kMaxGoalTrackAge);
        if (aiming_params.isPresent()) {
            return aiming_params;
        } else {
            Optional<AimingParameters> default_aiming_params = RobotState.getInstance().getDefaultAimingParameters();
            return default_aiming_params;
        }
    }

    /*** UPDATE VISION AIMING PARAMETERS FROM GOAL TRACKING ***/
    public void updateVisionAimingParameters() {
        // get aiming parameters from either vision-assisted goal tracking or
        // odometry-only tracking
        real_aiming_params_ = getRealAimingParameters();

        // predicted pose and target
        Pose2d predicted_field_to_vehicle = mRobotState
                .getPredictedFieldToVehicle(Constants.VisionConstants.kLookaheadTime);
        Pose2d predicted_vehicle_to_goal = predicted_field_to_vehicle.inverse()
                .transformBy(real_aiming_params_.get().getFieldToGoal());

        // update align delta from target and distance from target
        mTrackId = real_aiming_params_.get().getTrackId();
        mTargetAngle = predicted_vehicle_to_goal.getTranslation().direction().getRadians() + Math.PI;

        // send vision aligning target delta to swerve
        mSwerve.acceptLatestGoalTrackVisionAlignGoal(mTargetAngle);

        // update distance to target
        if (mLimelight.hasTarget() && mLimelight.getLimelightDistanceToTarget().isPresent()) {
            mCorrectedDistanceToTarget = mLimelight.getLimelightDistanceToTarget().get();
        } else {
            mCorrectedDistanceToTarget = predicted_vehicle_to_goal.getTranslation().norm();
        }
    }

    /*** UPDATE SHOOTER AND HOOD GOALS FROM DISTANCE ***/
    public synchronized void updateShootingSetpoints() {
        if (mPeriodicIO.SPIT) {
            mShooterSetpoint = kSpitVelocity;
            mHoodSetpoint = kSpitAngle;
        } else if (mPeriodicIO.FENDER) {
            mShooterSetpoint = kFenderVelocity;
            mHoodSetpoint = kFenderAngle;
        } else if (real_aiming_params_.isPresent()) {
            mShooterSetpoint = getShooterSetpointFromRegression(mCorrectedDistanceToTarget);
            mHoodSetpoint = getHoodSetpointFromRegression(mCorrectedDistanceToTarget);
        }

    }

    /***
     * UPDATE SUBSYSTEM STATES + SETPOINTS AND SET GOALS
     * 
     * 1. updates wanted actions for intake and indexer subsystems based on
     * requested superstructure action
     * 2. updates shooter and hood setpoint goals from tracked vars
     * 3. set subsystem states and shooting setpoints within subsystems
     * 
     */
    public void setGoals() {
        /* Update subsystem wanted actions and setpoints */

        // reset hood angle adjustment if called
        if (mResetHoodAngleAdjustment) {
            mHoodAngleAdjustment = 0.0;
            mResetHoodAngleAdjustment = false;
        }
        // update hood setpoint
        mPeriodicIO.real_hood = mHoodSetpoint + mHoodAngleAdjustment;

        // update shooter setpoint
        if (mPeriodicIO.PREP) {
            mPeriodicIO.real_shooter = mShooterSetpoint;
        } else {
            mPeriodicIO.real_shooter = 0.0;
        }

        // update intake and indexer actions
        if (mPeriodicIO.SPIT) {

            // only feed cargo to shooter when spun up
            if (isSpunUp()) {
                mPeriodicIO.real_trigger = Trigger.WantedAction.FEED;
                mIndexer.setWantFeeding(true);
            } else {
                mPeriodicIO.real_trigger = Trigger.WantedAction.NONE;
                mIndexer.setWantFeeding(false);
            }
        } else if (mPeriodicIO.SHOOT) {
            mPeriodicIO.real_intake = Intake.WantedAction.NONE;

            // only feed cargo to shooter when spun up
            if (isSpunUp()) {
                mIndexer.setWantFeeding(true);
                if (mPeriodicIO.FENDER) {
                    mPeriodicIO.real_trigger = Trigger.WantedAction.SLOW_FEED;
                } else {
                    mPeriodicIO.real_trigger = Trigger.WantedAction.FEED;
                }
            } else {
                mPeriodicIO.real_trigger = Trigger.WantedAction.NONE;
                mIndexer.setWantFeeding(false);
            }
        } else {
            mIndexer.setWantFeeding(false);
            mPeriodicIO.real_trigger = Trigger.WantedAction.NONE;

            mIndexer.setForceEject(mForceEject);
            mIndexer.setWantSlowEject(mSlowEject);

            // eject ball if we see a wrong color
            if (mColorSensor.seesBall() && !mColorSensor.hasCorrectColor()
                        && !indexerFull() && !mDisableEjecting) {
                mIndexer.queueEject();
            } else if (mColorSensor.seesNewBall()) {
                // otherwise just queue a ball for indexing
                if (!indexerFull()) {
                    mIndexer.queueBall(mColorSensor.hasCorrectColor());
                }
            }

            if (mPeriodicIO.INTAKE) {
                mPeriodicIO.real_intake = Intake.WantedAction.INTAKE;
            } else if (mPeriodicIO.REVERSE) {
                mPeriodicIO.real_intake = Intake.WantedAction.REVERSE;
            } else if (mPeriodicIO.REJECT) {
                mPeriodicIO.real_intake = Intake.WantedAction.REJECT;
            } else {
                mPeriodicIO.real_intake = Intake.WantedAction.NONE;
            }
        }

        // set intake state
        mIntake.setState(mPeriodicIO.real_intake);

        // set shooter subsystem setpoints
        if (Math.abs(mPeriodicIO.real_shooter) < Util.kEpsilon) {
            mShooter.setOpenLoop(0.0); // open loop if rpm goal is 0, to smooth spin down and stop belt skipping
        } else {
            mShooter.setVelocity(mShooterSetpoint);
        }
        mTrigger.setState(mPeriodicIO.real_trigger);

        // set hood subsystem setpoint
        // safety clamp the hood goal between min and max soft limits for hood angle
        mPeriodicIO.real_hood = Util.clamp(mPeriodicIO.real_hood,
                Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit,
                Constants.HoodConstants.kHoodServoConstants.kMaxUnitsLimit);

        if (mHood.mControlState != ServoMotorSubsystem.ControlState.OPEN_LOOP) {
            mHood.setSetpointMotionMagic(mPeriodicIO.real_hood);
        }
    }

    /*** UPDATE STATUS LEDS ON ROBOT ***/
    public void updateLEDs() {
        if (mLEDs.getUsingSmartdash()) {
            return;
        }

        State topState = State.OFF;
        State bottomState = State.OFF;

        if (hasEmergency) {
            topState = State.EMERGENCY;
            bottomState = State.EMERGENCY;
        } else {
            if (!mClimbMode) {
                if (getBallCount() == 2) {
                    bottomState = State.SOLID_GREEN;
                } else if (getBallCount() == 1) {
                    bottomState = State.SOLID_CYAN;
                } else {
                    bottomState = State.SOLID_ORANGE;
                }
                if (getWantsSpit()) {
                    topState = State.SOLID_ORANGE;
                } else if (getWantsFender()) {
                    topState = State.SOLID_CYAN;
                } else if (mPeriodicIO.SHOOT) {
                    topState = State.FLASHING_PINK;
                } else if (isAimed()) {
                    topState = State.FLASHING_GREEN;
                } else if (hasTarget()) {
                    topState = State.SOLID_PURPLE;
                } else {
                    topState = State.SOLID_ORANGE;
                }
            } else {
                if (mOpenLoopClimbControlMode) {
                    topState = State.SOLID_YELLOW;
                    bottomState = State.SOLID_YELLOW;
                } else if (mAutoTraversalClimb) {
                    topState = State.FLASHING_ORANGE;
                    bottomState = State.FLASHING_ORANGE;
                } else if (mAutoHighBarClimb) {
                    topState = State.FLASHING_CYAN;
                    bottomState = State.FLASHING_CYAN;
                } else {
                    topState = State.SOLID_PINK;
                    bottomState = State.SOLID_PINK;
                }
            }
        }

        mLEDs.applyStates(topState, bottomState);
    }

    // interpolates distance to target for shooter setpoint along regression
    private double getShooterSetpointFromRegression(double range) {
        if (ShooterRegression.kUseFlywheelAutoAimPolynomial) {
            return ShooterRegression.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    // interpolates distance to target for hood setpoint along regression
    private double getHoodSetpointFromRegression(double range) {
        if (ShooterRegression.kUseHoodAutoAimPolynomial) {
            return ShooterRegression.kHoodAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    // get vision align delta from goal
    public double getVisionAlignGoal() {
        return mTargetAngle;
    }

    // stop intaking if we have two of the correct cargo
    public boolean stopIntaking() {
        return (mIndexer.getTopBeamBreak() && mColorSensor.seesBall() && mColorSensor.hasCorrectColor());
    }

    // get number of correct cargo in indexer
    public double getBallCount() {
        return mBallCount;
    }

    // ball at back beam break and top beam break
    public boolean indexerFull() {
        return (mBallCount == 2);
    }

    // check if our flywheel is spun up to the correct velocity
    public boolean isSpunUp() {
        return mShooter.spunUp();
    }

    // check if our limelight sees a vision target
    public boolean hasTarget() {
        return mLimelight.hasTarget();
    }

    // checked if we are vision aligned to the target within an acceptable horiz. error
    public boolean isAimed() {
        return mLimelight.isAimed();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void stop() {
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.REVERSE = false;
        mPeriodicIO.REJECT = false;
        mPeriodicIO.PREP = false;
        mPeriodicIO.SHOOT = false;
        mPeriodicIO.FENDER = false;
        mPeriodicIO.SPIT = false;
        mHoodSetpoint = Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit + 1;
        mShooterSetpoint = 0.0;
        mForceEject = false;
    }

    /* Initial states for superstructure for teleop */
    public void setInitialTeleopStates() {
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.REVERSE = false;
        mPeriodicIO.REJECT = false;
        mPeriodicIO.PREP = true; // stay spun up
        mPeriodicIO.SHOOT = false;
        mPeriodicIO.FENDER = false;
        mPeriodicIO.SPIT = false;

        mClimbMode = false;

        System.out.println("Set initial teleop states!");
    }

    /* Superstructure getters for action and goal statuses */
    // get actions
    public boolean getIntaking() {
        return mPeriodicIO.INTAKE;
    }

    public boolean getReversing() {
        return mPeriodicIO.REVERSE;
    }

    public boolean getRejecting() {
        return mPeriodicIO.REJECT;
    }

    public boolean getEjecting() {
        return mIndexer.getIsEjecting();
    }

    public boolean getPrepping() {
        return mPeriodicIO.PREP;
    }

    public boolean getShooting() {
        return mPeriodicIO.SHOOT;
    }

    public boolean getWantsFender() {
        return mPeriodicIO.FENDER;
    }

    public boolean getWantsSpit() {
        return mPeriodicIO.SPIT;
    }

    public boolean getEjectDisabled() {
        return mDisableEjecting;
    }

    public boolean getIntakeOverride() {
        return mIntakeOverride;
    }

    // get other statuses
    public int getClimbStep() {
        return mClimbStep;
    }

    public boolean getInClimbMode() {
        return mClimbMode;
    }

    public boolean isOpenLoopClimbControl() {
        return mOpenLoopClimbControlMode;
    }

    public boolean isAutoClimb() {
        return mAutoTraversalClimb || mAutoHighBarClimb;
    }

    // get goals
    public String getIntakeGoal() {
        return mPeriodicIO.real_intake.toString();
    }

    public double getShooterGoal() {
        return mPeriodicIO.real_shooter;
    }

    public double getHoodGoal() {
        return mPeriodicIO.real_hood;
    }

    // included to continue logging while disabled
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mRoll = mPigeon.getRoll().getDegrees();
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "SUPERSTRUCTURE_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");
        headers.add("dt");
        headers.add("INTAKE");
        headers.add("REVERSE");
        headers.add("REJECT");
        headers.add("EJECT");
        headers.add("PREP");
        headers.add("SHOOT");
        headers.add("FENDER");
        headers.add("SPIT");
        headers.add("real_shooter");
        headers.add("real_hood");
        headers.add("gyro roll");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(mPeriodicIO.timestamp);
        items.add(mPeriodicIO.dt);
        items.add(mPeriodicIO.INTAKE ? 1.0 : 0.0);
        items.add(mPeriodicIO.REVERSE ? 1.0 : 0.0);
        items.add(mPeriodicIO.REJECT ? 1.0 : 0.0);
        items.add(mIndexer.getIsEjecting() ? 1.0 : 0.0);
        items.add(mPeriodicIO.PREP ? 1.0 : 0.0);
        items.add(mPeriodicIO.SHOOT ? 1.0 : 0.0);
        items.add(mPeriodicIO.FENDER ? 1.0 : 0.0);
        items.add(mPeriodicIO.SPIT ? 1.0 : 0.0);
        items.add(mPeriodicIO.real_shooter);
        items.add(mPeriodicIO.real_hood);
        items.add(mPigeon.getRoll().getDegrees());

        // send data to logging storage
        mStorage.addData(items);
    }

}
