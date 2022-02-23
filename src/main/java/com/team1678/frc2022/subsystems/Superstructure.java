package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.controlboard.ControlBoard;
import com.team1678.frc2022.controlboard.CustomXboxController;
import com.team1678.frc2022.controlboard.CustomXboxController.Button;
import com.team1678.frc2022.regressions.ShooterRegression;
import com.team254.lib.util.Util;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    // timer for reversing the intake and then stopping it once we have two correct cargo
    Timer mIntakeRejectTimer = new Timer();

    // PeriodicIO instance and paired csv writer
    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    
    /*** CONTAINER FOR SUPERSTRUCTURE ACTIONS AND GOALS ***/
    public static class PeriodicIO {
        // INPUTS
        // (superstructure actions)
        private boolean INTAKE = false; // run the intake to pick up cargo
        private boolean REVERSE = false; // reverse the intake and singulator
        private boolean REJECT = false; // have the intake reject cargo
        private boolean EJECT = false; // run pooper
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
        private Indexer.WantedAction real_indexer = Indexer.WantedAction.NONE;
        private Trigger.WantedAction real_trigger = Trigger.WantedAction.NONE;
        private double real_shooter = 0.0;
        private double real_hood = 0.0;   
    }

    /* Setpoint Tracker Variables */
    // shooting system setpoints
    public double mShooterSetpoint = 1000.0;
    public double mHoodSetpoint = 20.0; // TODO: arbitrary value, change4
    private double mHoodAngleAdjustment = 0.0;
    private boolean mResetHoodAngleAdjustment = false;

    // intake / eject locking status
    private boolean mLockIntake = false;
    private boolean mForceIntake = false;
    private boolean mForceEject = false;
    private boolean mDisableEjecting = false;

    // climb mode tracker variables
    private boolean mClimbMode = false;
	private boolean mOpenLoopClimbControlMode = false;
	private boolean mResetClimberPosition = false;
    private boolean mAutoTraversalClimb = false;
    private int mClimbStep = 0;

    // fender shot constants
    private final double kFenderVelocity = 2300;
    private final double kFenderAngle = 12.0;

    private final double kSpitVelocity = 1000;
    private final double kSpitAngle = 20.0;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();

                updateShootingParams();
                setGoals();
                outputTelemetry();

                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
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
    public void setWantEject(boolean eject) {
        mPeriodicIO.EJECT = eject;
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

    /*** CONTAINER FOR OPERATOR COMMANDS CALLING SUPERSTRUCTURE ACTIONS
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
     *   --> 0 to move hood up
     *   --> 180 to move hood down
     * - press START button to reset adjustment
     * 
     * Other Manual Sets
     * - press dpad left (POV 270) to toggle force intake
     * - press dpad right (POV 90) to toggle disabling the ejector
     * - hold left bumper to eject balls manuallyo
     * 
     * Climb Controls
     * - press left bumper, right bumper, left trigger, right trigger to enter climb mode
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
     * */
    public void updateOperatorCommands() {
          
        // get whether we want to enter climb mode
        if (mControlBoard.getClimbMode()) {
            mClimbMode = true;
        }

        if (mClimbMode) {

            /*** CLIMB MODE CONTROLS ***/

            // stop all other superstructure actions
            stop();

            if (mControlBoard.getExitClimbMode()) {
                mClimbMode = false;
            }

            if (mControlBoard.operator.getController().getLeftStickButtonPressed()) {
                mOpenLoopClimbControlMode = !mOpenLoopClimbControlMode;
                mClimber.setClimberNone();
            }

            if (mControlBoard.operator.getController().getRightStickButtonPressed()) {
                mResetClimberPosition = true;
            }

            if (mResetClimberPosition) {
                mClimber.resetClimberPosition();
                mResetClimberPosition = false;
            }

            if (!mOpenLoopClimbControlMode) {

                if (mControlBoard.operator.getController().getXButtonPressed()) {
                    mClimber.setClimberNone();
                    mClimbStep = 0;
                    
                } else if (mControlBoard.operator.getController().getAButtonPressed()) {
                    mClimber.setExtendForClimb();

                } else if (mControlBoard.operator.getController().getBButtonPressed()) {
                    mClimber.setClimbMidBar();

                } else if (mControlBoard.operator.getController().getPOV() == 180) {
                    mClimber.setClimbMidBarAndExtend();

                } else if (mControlBoard.operator.getController().getPOV() == 90) {
                    mClimber.setClimbHighBarAndExtend();

                } else if (mControlBoard.operator.getController().getPOV() == 0) {
                    mClimber.setTraversalBarExtend();
                
                } else if (mControlBoard.operator.getController().getPOV() == 270) {
                    mClimber.setClimbTraversalBar();

                } else if (mControlBoard.getTraversalClimb()) {
                    mAutoTraversalClimb = !mAutoTraversalClimb;
                }
                
                if (mAutoTraversalClimb) {

                    // climb mid bar and extend to high bar
                    if (mClimbStep == 0) {
                        mClimber.setClimbMidBarAndExtend();
                        mClimbStep++; // climb step 1
                    }

                    // set left arm to full extension from partial height to make contact on high bar
                    if (mSwerve.getRoll().getDegrees() < Constants.ClimberConstants.kHighBarExtendAngle // check if dt roll is past high bar while swinging to extend
                        &&
                        Util.epsilonEquals(mClimber.getClimberPositionLeft(), // don't extend unless left arm is at partial height
                                            Constants.ClimberConstants.kLeftPartialTravelDistance,
                                            Constants.ClimberConstants.kTravelDistanceEpsilon)
                        && (mClimbStep == 1)) {

                        mClimber.setHighBarExtend();
                        mClimbStep++; // climb step 2
                    }

                    // pull up with left arm on upper bar while extending right arm to traversal bar
                    if ((mSwerve.getRoll().getDegrees() > Constants.ClimberConstants.kHighBarContactAngle) // check if dt roll is at bar contact angle before climbing to next bar
                        &&
                        Util.epsilonEquals(mClimber.getClimberPositionLeft(), // don't climb unless left arm is fully extended
                                            Constants.ClimberConstants.kLeftTravelDistance,
                                            Constants.ClimberConstants.kTravelDistanceEpsilon)
                        &&
                        Util.epsilonEquals(mClimber.getClimberPositionRight(), // don't climb unless left arm is fully extended
                                            Constants.ClimberConstants.kSafetyMinimum,
                                            Constants.ClimberConstants.kTravelDistanceEpsilon)
                        && (mClimbStep == 2)) {

                        mClimber.setClimbHighBarAndExtend();
                        mClimbStep++; // climb step 3
                    }

                    // set right arm to full extension from partial height to make contact on traversal bar
                    if (mSwerve.getRoll().getDegrees() > Constants.ClimberConstants.kTraversalBarExtendAngle // check if dt roll is past traversal bar while swinging to extend
                        &&
                        Util.epsilonEquals(mClimber.getClimberPositionRight(), // don't extend unless right arm is at partial height
                                            Constants.ClimberConstants.kRightPartialTravelDistance,
                                            Constants.ClimberConstants.kTravelDistanceEpsilon)
                        && (mClimbStep == 3)) {

                        mClimber.setTraversalBarExtend();
                        mClimbStep++; // climb step 4
                    }

                    // climb on the right arm after we are fully extended on traversal bar
                    if (Util.epsilonEquals(mSwerve.getRoll().getDegrees(), // check if dt roll is at the angle necessary 
                                            Constants.ClimberConstants.kTraversalBarContactAngle,
                                            2.0)
                        &&
                        Util.epsilonEquals(mClimber.getClimberPositionRight(), // don't extend unless right arm is at full height
                                            Constants.ClimberConstants.kRightTravelDistance,
                                            Constants.ClimberConstants.kTravelDistanceEpsilon)
                        && (mClimbStep == 4)) {

                        mClimber.setClimbTraversalBar();
                        mClimbStep++; // climb step 5
                    }
                }

            } else {

                // set left climber motor open loop
                if (mControlBoard.operator.getController().getPOV() == 0) {
                    mClimber.setLeftClimberOpenLoop(8.0);
                } else if (mControlBoard.operator.getController().getPOV() == 180) {
                    mClimber.setLeftClimberOpenLoop(-8.0);
                } else {
                    mClimber.setLeftClimberOpenLoop(0.0);
                }

                // set right climber motor open loop
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

            // toggle whether we want to force intake or decide whether we intake based on whether we have two correct cargo
            if (mControlBoard.operator.getController().getPOV() == 270) {
                mForceIntake = !mForceIntake;
            }
            // control intake + reverse actions
            if (mForceIntake) {
                normalIntakeControls();
            } else {
                // start a timer for rejecting balls and then locking the intake when we want to stop intaking
                if (stopIntaking()) {
                    mLockIntake = true;
                    mIntakeRejectTimer.reset();
                    mIntakeRejectTimer.start();
                }

                // if we want to lock the intake, reject incoming cargo for a short time and then lock the intake
                if (mLockIntake) {
                    if (mIntakeRejectTimer.hasElapsed(Constants.IntakeConstants.kIntakeRejectTime)) {
                        mIntakeRejectTimer.stop();
                        setWantIntakeNone();
                    } else {
                        setWantReject(true);
                    }

                    // if:
                    // - we don't have a ball indexed and a ball in our system
                    // - we don't have a ball at either fully indexed position
                    // - we don't want to stop intaking
                    // then unlock the intake
                    if (!(mIndexer.getTopBeamBreak() && mColorSensor.hasBall())
                            && !indexerFull()
                            && !stopIntaking()) {
                        
                        mLockIntake = false;
                    }
                } else {
                    normalIntakeControls();
                }
            }            

            // toggle ejecting to disable if necessary
            if (mControlBoard.operator.getController().getPOV() == 90) {
                mDisableEjecting = !mDisableEjecting;
            }
            // control options to filter cargo and eject
            // don't eject if we want it disabled or if we lock the intake because we have two correct cargo
            if (mDisableEjecting || mLockIntake) {
                mPeriodicIO.EJECT = false;
            } else if (mControlBoard.operator.getButton(Button.LB)) {
                mPeriodicIO.EJECT = true;
                mForceEject = true;
            } else {
                mForceEject = false;
                // when not forcing an eject, passively check whether want to passively eject using color sensor logic
                mPeriodicIO.EJECT = mColorSensor.wantsEject();
            }

            // control shooting
            if (mControlBoard.operator.getController().getYButtonPressed()) {
                mPeriodicIO.SHOOT = !mPeriodicIO.SHOOT;
            }

            // control prepping
            if (mControlBoard.operator.getController().getAButtonPressed()) {
                mPeriodicIO.PREP = !mPeriodicIO.PREP;
            }

            // control fender shot
            if (mControlBoard.operator.getController().getBButtonPressed()) {
                mPeriodicIO.FENDER = !mPeriodicIO.FENDER;
            }

            // control spit shot
            if (mControlBoard.operator.getController().getXButtonPressed()) {
                mPeriodicIO.SPIT = !mPeriodicIO.SPIT;
            }

            // control for adding manual hood adjustment
            switch(mControlBoard.getHoodManualAdjustment()) {
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
            if (mControlBoard.operator.getButton(CustomXboxController.Button.START)) {
                mResetHoodAngleAdjustment = true;
            }
        }
    }

    /*** UPDATE SHOOTER AND HOOD SETPOINTS WHEN VISION AIMING ***/
    public synchronized void updateShootingParams() {
        if (mPeriodicIO.SPIT) {
            mShooterSetpoint = kSpitVelocity;
            mHoodSetpoint = kSpitAngle;
        } else if (mPeriodicIO.FENDER) {
            mShooterSetpoint = kFenderVelocity;
            mHoodSetpoint = kFenderAngle;
        } else if (hasTarget()) {
            Optional<Double> distance_to_target = mLimelight.getDistanceToTarget();
            if (distance_to_target.isPresent()) {
                mShooterSetpoint = getShooterSetpointFromRegression(distance_to_target.get());
                mHoodSetpoint = getHoodSetpointFromRegression(distance_to_target.get());
            }
        }
    }

    /*** UPDATE SUBSYSTEM STATES + SETPOINTS AND SET GOALS
     * 
     * 1. updates wanted actions for intake and indexer subsystems based on requested superstructure action
     * 2. updates shooter and hood setpoint goals from tracked vars
     * 3. set subsystem states and shooting setpoints within subsystems
     * 
     * */
    public void setGoals() {
        /* Update subsystem wanted actions and setpoints*/

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
        if (mPeriodicIO.SHOOT) {
            mPeriodicIO.real_intake = Intake.WantedAction.NONE;

            // only feed cargo to shoot when spun up and aimed
            if (isSpunUp() /*&& isAimed()*/) {
                mPeriodicIO.real_indexer = Indexer.WantedAction.FEED;
                if (mPeriodicIO.FENDER) {
                    mPeriodicIO.real_trigger = Trigger.WantedAction.SLOW_FEED;
                } else {
                    mPeriodicIO.real_trigger = Trigger.WantedAction.FEED;
                }
            } else {
                mPeriodicIO.real_trigger = Trigger.WantedAction.NONE;
                mPeriodicIO.real_indexer = Indexer.WantedAction.NONE;
            }
        } else {
            // force eject
            if (mPeriodicIO.EJECT && mForceEject) {
                mPeriodicIO.real_indexer = Indexer.WantedAction.EJECT;
                mPeriodicIO.real_trigger = Trigger.WantedAction.PASSIVE_REVERSE;
            // only do any indexing action if we detect a ball
            } else if (mColorSensor.hasBall()) {
                mPeriodicIO.real_trigger = Trigger.WantedAction.PASSIVE_REVERSE;
                if (mPeriodicIO.EJECT) {
                    mPeriodicIO.real_indexer = Indexer.WantedAction.EJECT;
                } else {
                    mPeriodicIO.real_indexer = Indexer.WantedAction.INDEX;
                }
            } else {
                mPeriodicIO.real_trigger = Trigger.WantedAction.NONE;
                mPeriodicIO.real_indexer = Indexer.WantedAction.NONE;
            }

            // normal operator manual control for intake
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

        /* Set subsystem states + setpoints based on wanted actions */

        // set intake and indexer states
        mIntake.setState(mPeriodicIO.real_intake);
        mIndexer.setState(mPeriodicIO.real_indexer);

        // set shooter subsystem setpoint
        if (Math.abs(mPeriodicIO.real_shooter) < Util.kEpsilon) {
            mShooter.setOpenLoop(0, 0); // open loop if rpm goal is 0, to smooth spin down and stop belt skipping
        } else {
            mShooter.setVelocity(mShooterSetpoint, mShooterSetpoint * Constants.ShooterConstants.kAcceleratorMultiplier);
        }
        mTrigger.setState(mPeriodicIO.real_trigger);

        // set hood subsystem setpoint
        // safety clamp the hood goal between min and max hard stops for hood angle
        mPeriodicIO.real_hood = Util.clamp(mPeriodicIO.real_hood,
                Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit,
                Constants.HoodConstants.kHoodServoConstants.kMaxUnitsLimit); 
        if (mHood.mControlState != ServoMotorSubsystem.ControlState.OPEN_LOOP) {
            mHood.setSetpointMotionMagic(mPeriodicIO.real_hood);
        }
    }

    /*** GET SHOOTER AND HOOD SETPOINTS FROM SUPERSTRUCTURE CONSTANTS REGRESSION ***/
    // interpolates distance to target for shooter setpoint along regression
    private double getShooterSetpointFromRegression(double range) {
        if (ShooterRegression.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Shooting RPM", 0);
        } else if (ShooterRegression.kUseFlywheelAutoAimPolynomial) {
            return ShooterRegression.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }
    // interpolates distance to target for hood setpoint along regression
    private double getHoodSetpointFromRegression(double range) {
        if (ShooterRegression.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Hood Angle", 0);
        } else if (ShooterRegression.kUseHoodAutoAimPolynomial) {
            return ShooterRegression.kHoodAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    // call normal intake controls
    public void normalIntakeControls() {
        if (mControlBoard.operator.getTrigger(CustomXboxController.Side.RIGHT)) {
            setWantIntake(true);
        } else if (mControlBoard.operator.getTrigger(CustomXboxController.Side.LEFT)) {
            setWantReverse(true);
        } else {
            setWantIntakeNone();
        }
    }
    // stop intaking if we have two of the correct cargo
    public boolean stopIntaking() {
        return (mIndexer.getTopBeamBreak() && mColorSensor.seesBall() && mColorSensor.hasCorrectColor());
    }
    // ball at back beam break and top beam break
    public boolean indexerFull() {
        return (mIndexer.getTopBeamBreak() && mIndexer.getBottomBeamBreak());
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
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void stop() {
        mPeriodicIO.INTAKE = false;
        mPeriodicIO.REVERSE = false;
        mPeriodicIO.REJECT = false;
        mPeriodicIO.EJECT = false;
        mPeriodicIO.PREP = false;
        mPeriodicIO.SHOOT = false;
        mPeriodicIO.FENDER = false;
        mPeriodicIO.SPIT = false;

        mHoodSetpoint = Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit + 1;
        mShooterSetpoint = 0.0;
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
        return mPeriodicIO.EJECT;
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

    // get other statuses
    public boolean getInClimbMode() {
        return mClimbMode;
    }
    public boolean isOpenLoopClimbControl() {
        return mOpenLoopClimbControlMode;
    }

    // get goals
    public String getIntakeGoal() {
        return mPeriodicIO.real_intake.toString();
    }
    public String getIndexerGoal() {
        return mPeriodicIO.real_indexer.toString();
    }
    public double getShooterGoal() {
        return mPeriodicIO.real_shooter;
    }
    public double getHoodGoal() {
        return mPeriodicIO.real_hood;
    }

    /* Output superstructure actions and other related statuses */
    public void outputTelemetry() {
        // // superstructure actions requested
        // SmartDashboard.putBoolean("Intaking", mPeriodicIO.INTAKE);
        // SmartDashboard.putBoolean("Reversing", mPeriodicIO.REVERSE);
        // SmartDashboard.putBoolean("Ejecting", mPeriodicIO.EJECT);
        // SmartDashboard.putBoolean("Prepping", mPeriodicIO.PREP);
        // SmartDashboard.putBoolean("Shooting", mPeriodicIO.SHOOT);
        // SmartDashboard.putBoolean("Fender Shooting", mPeriodicIO.FENDER);

        // // superstructure goals being set
        // SmartDashboard.putString("Intake Goal", mPeriodicIO.real_intake.toString());
        // SmartDashboard.putString("Indexer Goal", mPeriodicIO.real_indexer.toString());
        // SmartDashboard.putNumber("Shooter Goal", mPeriodicIO.real_shooter);
        // SmartDashboard.putNumber("Hood Goal", mPeriodicIO.real_hood);

        // // other status variables
        // SmartDashboard.putNumber("Superstructure dt", mPeriodicIO.dt);
        // SmartDashboard.putBoolean("Is Spun Up", isSpunUp());
        // SmartDashboard.putBoolean("Has Vision Target", mLimelight.hasTarget());
        // SmartDashboard.putBoolean("Is Vision Aimed", mLimelight.isAimed());
       
        SmartDashboard.putBoolean("Disable Ejecting", mDisableEjecting);
        SmartDashboard.putBoolean("Stop Intaking", stopIntaking());
        SmartDashboard.putBoolean("Force Intake", mForceIntake);
        SmartDashboard.putBoolean("Force Eject", mForceEject);

        SmartDashboard.putBoolean("Auto Traversal Climb", mAutoTraversalClimb);
        SmartDashboard.putNumber("Climb Step Number", mClimbStep);
    }

    // included to continue logging while disabled
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        // write inputs and ouputs from PeriodicIO to csv 
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    // instantiate csv writer
    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/SUPERSTRUCTURE-LOGS.csv", PeriodicIO.class);
        }
    }

    // send written csv data to file and end log
    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
    
}
