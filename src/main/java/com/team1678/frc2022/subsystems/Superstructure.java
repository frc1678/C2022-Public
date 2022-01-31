package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.regressions.ShooterRegression;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.controlboard.ControlBoard;
import com.team1678.frc2022.controlboard.CustomXboxController.Side;
import com.team1678.frc2022.loops.ILooper;

public class Superstructure extends Subsystem {

    /* Superstructure Instance */
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    /*** REQUIRED INSTANCES ***/
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();

    // PeriodicIO instance and paired csv writer
    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    
    /*** CONTAINER FOR SUPERSTRUCTURE ACTIONS AND GOALS ***/
    public static class PeriodicIO {
        // INPUTS
        // (superstructure actions)
        public boolean NONE = false; // do nothing
        public boolean INTAKE = false; // run the intake to pick up cargo
        public boolean OUTTAKE = false; // reverse the intake to spit out cargo
        public boolean PREP = false; // spin up and aim with shooting setpoints
        public boolean SHOOT = false; // shoot cargo
        public boolean FENDOR = false; // shoot cargo from up against the hub

        // time measurements
        public double timestamp;
        public double dt;

        // OUTPUTS
        // (superstructure goals/setpoints)
        Intake.WantedAction real_intake;
        Indexer.WantedAction real_indexer;
        double real_shooter;
        double real_hood;
        
    }

    // setpoint tracker variables
    public double mShooterSetpoint = 0.0;
    public double mHoodSetpoint = 10.0; // TODO: arbitrary value, change

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

                updateOperatorCommands();
                maybeUpdateGoalFromVision();
                setGoals();
                outputTelemetry();

                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stopLogging();
            }
        });
    }

    /*** CONTAINER FOR OPERATOR COMMANDS CALLING SUPERSTRUCTURE ACTIONS ***/
    public void updateOperatorCommands() {
        // control intake vs. outtake actions
        if (mControlBoard.operator.getTrigger(Side.RIGHT)) {
            mPeriodicIO.INTAKE = true;
        } else if (mControlBoard.operator.getTrigger(Side.LEFT)) {
            mPeriodicIO.OUTTAKE = true;
        } else {
            mPeriodicIO.INTAKE = false;
            mPeriodicIO.OUTTAKE = false;
        }

        // control shooting
        if (mControlBoard.operator.getController().getYButtonPressed()) {
            mPeriodicIO.SHOOT = !mPeriodicIO.SHOOT;
        }

        // control prepping
        if (mControlBoard.operator.getController().getAButtonPressed()) {
            mPeriodicIO.PREP = !mPeriodicIO.PREP;
        }
    }

    /*** SETTERS FOR SUPERSTRUCTURE ACTIONS OUTSIDE OPERATOR INPUT ***/
    public void setWantIntake(boolean intake) {
        if (mPeriodicIO.INTAKE != intake) {
            mPeriodicIO.INTAKE = intake;
        }
    }
    public void setWantOuttake(boolean outtake) {
        if (mPeriodicIO.OUTTAKE != outtake) {
            mPeriodicIO.OUTTAKE = outtake;
        }
    }
    public void setWantPrep(boolean wants_prep) {
        mPeriodicIO.PREP = wants_prep;
    }
    public void setWantShoot(boolean wants_shoot) {
        mPeriodicIO.SHOOT = wants_shoot;
    }
    public void setShooterVelocity(double velocity) {
        mShooterSetpoint = velocity;
    }
    public void setWantIntake() {
        setWantIntake(!mPeriodicIO.INTAKE);
    }

    /*** UPDATE SHOOTER AND HOOD SETPOINTS WHEN VISION AIMING ***/
    public synchronized void maybeUpdateGoalFromVision() {
        if (mLimelight.hasTarget()) {
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
    */
    public void setGoals() {
        /* Update subsystem wanted actions and setpoints*/

        // update hood setpoint
        mPeriodicIO.real_hood = mHoodSetpoint;

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
            if (isSpunUp() && isAimed()) {
                mPeriodicIO.real_indexer = Indexer.WantedAction.FEED;
            } else {
                mPeriodicIO.real_indexer = Indexer.WantedAction.INDEX;
            }
        } else {
            if (mPeriodicIO.INTAKE) {
                mPeriodicIO.real_intake = Intake.WantedAction.INTAKE;
                mPeriodicIO.real_indexer = Indexer.WantedAction.INDEX;
            } else if (mPeriodicIO.OUTTAKE) {
                mPeriodicIO.real_intake = Intake.WantedAction.REVERSE;
                mPeriodicIO.real_indexer = Indexer.WantedAction.REVERSE;
            } else {
                mPeriodicIO.real_intake = Intake.WantedAction.NONE;
                mPeriodicIO.real_indexer = Indexer.WantedAction.INDEX; // always in indexing state
            }
        }

        /* Set subsystem states + setpoints based on wanted actions */

        // set intake and indexer states
        mIntake.setState(mPeriodicIO.real_intake);
        mIndexer.setState(mPeriodicIO.real_indexer);

        // set shooter subsystem setpoint
        if (Math.abs(mPeriodicIO.real_shooter) < Util.kEpsilon) {
            mShooter.setOpenLoop(0); // open loop if rpm goal is 0, to smooth spin down and stop belt skipping
        } else {
            mShooter.setVelocity(mShooterSetpoint);
        }

        // set hood subsystem setpoint
        // safety clamp the hood goal between min and max hard stops for hood angle
        mPeriodicIO.real_hood = Util.clamp(mPeriodicIO.real_hood,
                Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit,
                Constants.HoodConstants.kHoodServoConstants.kMaxUnitsLimit); 
        mHood.setSetpointMotionMagic(mPeriodicIO.real_hood);
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

    // status checker methods
    public boolean isSpunUp() {
        return mShooter.spunUp();
    }
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
        // TODO Auto-generated method stub
    }

    /* Output superstructure actions and other related statuses */
    public void outputTelemetry() {
        // superstructure action statuses
        SmartDashboard.putBoolean("Intaking", mPeriodicIO.INTAKE);
        SmartDashboard.putBoolean("Outtaking", mPeriodicIO.OUTTAKE);
        SmartDashboard.putBoolean("Prepping", mPeriodicIO.PREP);
        SmartDashboard.putBoolean("Shooting", mPeriodicIO.SHOOT);
        SmartDashboard.putBoolean("Fendor Shooting", mPeriodicIO.FENDOR);

        // other status variables
        SmartDashboard.putNumber("Superstructure dt", mPeriodicIO.dt);
        SmartDashboard.putBoolean("Is Spun Up", isSpunUp());
        SmartDashboard.putBoolean("Has Vision Target", mLimelight.hasTarget());
        SmartDashboard.putBoolean("Is Vision Aimed", mLimelight.isAimed());
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
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/INDEXER-LOGS.csv", PeriodicIO.class);
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
