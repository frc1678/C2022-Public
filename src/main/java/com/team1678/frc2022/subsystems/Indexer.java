package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;


public class Indexer extends Subsystem {
    
    private final TalonFX mIndexer;
    private final TalonFX mTrigger;

    private static Indexer mInstance;
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final DigitalInput mBottomBeamBreak;
    private final DigitalInput mTopBeamBreak;

    private State mState = State.IDLE;

    private boolean mBottomHadSeenBall = false;
    private boolean mTopHadSeenBall = false;

    private final double kReverseAtTopTime = 1.0;
    private Timer mReverseTriggerTimer = new Timer();
    private boolean mWantTriggerReverse = false;

    public enum WantedAction {
        NONE,
        INDEX,
        FEED,
        REVERSE
    }

    public enum State {
        IDLE,
        INDEXING,
        FEEDING,
        REVERSING
    }

    private Indexer() {
        //mSuperstructure = Superstructure.getInstance();

        mIndexer = TalonFXFactory.createDefaultTalon(Ports.HOPPER_ID);
        mTrigger = TalonFXFactory.createDefaultTalon(Ports.TRIGGER_ID);
        mTrigger.setInverted(true);
      
        mBottomBeamBreak = new DigitalInput(Ports.BOTTOM_BEAM_BREAK);
        mTopBeamBreak = new DigitalInput(Ports.TOP_BEAM_BREAK);
    }

    public static synchronized Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public synchronized State getState() {
        return mState;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.top_break = !mBottomBeamBreak.get();
        mPeriodicIO.bottom_break = !mTopBeamBreak.get();

        mPeriodicIO.trigger_current = mTrigger.getStatorCurrent();
        mPeriodicIO.trigger_voltage = mTrigger.getMotorOutputVoltage();

        mPeriodicIO.tunnel_current = mIndexer.getStatorCurrent();
        mPeriodicIO.tunnel_voltage = mIndexer.getMotorOutputVoltage();

        if (mPeriodicIO.bottom_break) {
            if (!mBottomHadSeenBall) {
                mBottomHadSeenBall = true;
            }
        } else {
            if (mBottomHadSeenBall) {
                mPeriodicIO.ball_count++;
                mBottomHadSeenBall = false;
            }
        }

        if (mPeriodicIO.top_break) {
            if (!mTopHadSeenBall) {
                mTopHadSeenBall = true;
            }
        } else {
            if (mTopHadSeenBall) {
                mPeriodicIO.ball_count--;
                mTopHadSeenBall = false;
            }
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mIndexer.set(ControlMode.PercentOutput, mPeriodicIO.tunnel_demand / 12.0);
        mTrigger.set(ControlMode.PercentOutput, mPeriodicIO.trigger_demand / 12.0);
    }

    @Override
    public void registerEnabledLoops (ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Indexer.this) {
                    runStateMachine();
                }
                outputTelemetry();
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stop();
            }
        });
    }

    /**
     * Gets the current status of the top beam break
     * @return the state of the beam break
     */
    public boolean getTopBeamBreak() {
        return mPeriodicIO.top_break;
    }

    /**
     * Gets the current status of the top beam break
     * @return the state of the beam break
     */
    public boolean getBottomBeamBreak() {
        return mPeriodicIO.bottom_break;
    }

    public double getTunnelDemand() {
        return mPeriodicIO.tunnel_demand;
    }

    public double getTunnelCurrent() {
        return mPeriodicIO.tunnel_current;
    }
    
    public double getTunnelVoltage() {
        return mPeriodicIO.tunnel_voltage;
    }

    public double getTriggerDemand() {
        return mPeriodicIO.trigger_demand;
    }

    public double getTriggerCurrent() {
        return mPeriodicIO.trigger_current;
    }
    
    public double getTriggerVoltage() {
        return mPeriodicIO.trigger_voltage;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INDEX:
                mState = State.INDEXING;
                break;
            case FEED:
                mState = State.FEEDING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
        }
    }

    private boolean ballAtTrigger() {
        return mPeriodicIO.top_break;
    }

    private boolean ballAtTunnel() {
        return mPeriodicIO.bottom_break;
    }

    private boolean stopTunnel() {
        if (mPeriodicIO.ball_count <= 1) {
            return false;
        } else
            return true;
    }

    private boolean runTrigger() {
        return !ballAtTrigger();
    }

    private void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kIdleVoltage;
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kIdleVoltage;
                break;
            case INDEXING:
                if (Ejector.getInstance().shouldStopHoppper()) {
                    mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kIdleVoltage;
                    mPeriodicIO.trigger_demand = Constants.IndexerConstants.kIdleVoltage;
                } else {
                    if (runTrigger()) {
                        mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerIndexingVoltage;
                    } else { 
                        mPeriodicIO.trigger_demand = Constants.IndexerConstants.kIdleVoltage;
                    }

                    /*
                    if (ballAtTrigger()) {
                        mReverseTriggerTimer.start();
                        mWantTriggerReverse = true;
                    }

                    if (!mReverseTriggerTimer.hasElapsed(kReverseAtTopTime) && mWantTriggerReverse) {
                        mPeriodicIO.trigger_demand = -Constants.IndexerConstants.kTriggerIndexingVoltage;    
                    } else if (mReverseTriggerTimer.hasElapsed(kReverseAtTopTime) && mWantTriggerReverse){
                        mPeriodicIO.trigger_demand = Constants.IndexerConstants.kIdleVoltage;
                        mWantTriggerReverse = false;
                        mReverseTriggerTimer.reset();
                        mPeriodicIO.ball_count++;
                    } else {
                        if (mPeriodicIO.ball_count > 0) {
                            mPeriodicIO.trigger_demand = Constants.IndexerConstants.kIdleVoltage;
                        } else {
                            mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerIndexingVoltage;
                        }
                    }
                    */

                    if (stopTunnel()) {
                        mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kIdleVoltage;
                    } else {
                        mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kIndexerIndexingVoltage;
                    }
                }
                break;
            case FEEDING:
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kFeedingVoltage;
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kFeedingVoltage;
                break;
            case REVERSING:
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerReversingVoltage;
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kIndexerReversingVoltage;
                break;

        }
    }
    
    @Override
    public void stop() {
        //mTrigger.set(ControlMode.PercentOutput, 0);
        mIndexer.set(ControlMode.PercentOutput, 0);

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;

        public double tunnel_voltage;
        public double tunnel_current;
        public double trigger_voltage;
        public double trigger_current;

        public boolean top_break;
        public boolean bottom_break;
        public boolean correctColor;
        public double ball_count;

        //OUTPUTS
        public double tunnel_demand;
        public double trigger_demand;
        public boolean eject;
    }

    public void outputTelemetry() {
        SmartDashboard.putBoolean("Top had seen ball", mTopHadSeenBall);
        SmartDashboard.putBoolean("Bottom had seen ball", mBottomHadSeenBall);

        SmartDashboard.putNumber("Ball count", mPeriodicIO.ball_count);
    }
}
