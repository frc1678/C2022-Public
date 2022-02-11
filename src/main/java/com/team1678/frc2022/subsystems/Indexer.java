package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {
    
    private final TalonFX mTunnel;
    private final TalonFX mTrigger;

    private static Indexer mInstance;
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final DigitalInput mBottomBeamBreak;
    private final DigitalInput mTopBeamBreak;
    private final Timer mEmptyIndexerTimer = new Timer();
    //TODO: private final DigitalInput mColorSensor = new DigitalInput(Ports.COLOR_SENOR);

    private State mState = State.IDLE;

    private boolean mBottomHadSeenBall = false;
    private boolean mTopHadSeenBall = false;
    private boolean mWasReversing = false;

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
        REVERSING,
    }

    private Indexer() {

        /* Trigger Motor */
        mTrigger = TalonFXFactory.createDefaultTalon(Ports.TRIGGER_ID);
        mTrigger.setInverted(true);
        mTrigger.setNeutralMode(NeutralMode.Brake);

        // Closed Loop Tuning
        mTrigger.config_kI(0, Constants.IndexerConstants.kTriggerI, Constants.kLongCANTimeoutMs);
        mTrigger.config_kD(0, Constants.IndexerConstants.kTriggerD, Constants.kLongCANTimeoutMs);
        mTrigger.config_kP(0, Constants.IndexerConstants.kTriggerP, Constants.kLongCANTimeoutMs);
        mTrigger.config_kF(0, Constants.IndexerConstants.kTriggerF, Constants.kLongCANTimeoutMs);
        mTrigger.selectProfileSlot(0, 0);

        // Use Integrated Encoder
        mTrigger.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        
        // Reduce CAN Util
        mTrigger.changeMotionControlFramePeriod(255);
        mTrigger.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mTrigger.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);

        /* Tunnel Motor */
        mTunnel = TalonFXFactory.createDefaultTalon(Ports.TUNNEL_ID);

        // Closed Loop Tuning
        mTunnel.config_kI(0, Constants.IndexerConstants.kTunnelI, Constants.kLongCANTimeoutMs);
        mTunnel.config_kD(0, Constants.IndexerConstants.kTunnelD, Constants.kLongCANTimeoutMs);
        mTunnel.config_kP(0, Constants.IndexerConstants.kTunnelP, Constants.kLongCANTimeoutMs);
        mTunnel.config_kF(0, Constants.IndexerConstants.kTunnelF, Constants.kLongCANTimeoutMs);
        mTunnel.selectProfileSlot(0, 0);

        // Use Integrated Encoder
        mTunnel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        // Reduce CAN Util
        mTunnel.changeMotionControlFramePeriod(255);
        mTunnel.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mTunnel.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
      
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
        mPeriodicIO.trigger_velocity = mTrigger.getSelectedSensorVelocity() * Constants.IndexerConstants.kTriggerVelocityConversion;
        
        mPeriodicIO.tunnel_current = mTunnel.getStatorCurrent();
        mPeriodicIO.tunnel_voltage = mTunnel.getMotorOutputVoltage();
        mPeriodicIO.tunnel_velocity = mTunnel.getSelectedSensorVelocity() * Constants.IndexerConstants.kTunnelVelocityConversion;
    }

    @Override
    public void writePeriodicOutputs() {
        mTunnel.set(ControlMode.PercentOutput, mPeriodicIO.tunnel_demand / 12.0);
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
                    updateBallCounter();
                    runStateMachine();
                }
                // outputTelemetry();
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

    public double getTunnelVelocity() {
        return mPeriodicIO.tunnel_velocity;
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

    public double getTriggerVelocity() {
        return mPeriodicIO.trigger_velocity;
    }

    public double getBallCount() {
        return mPeriodicIO.ball_count;
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
        if ((ballAtTunnel()) || (mPeriodicIO.forceTunnelOn && !ballAtTunnel())) {
            return false;
        } else
            return true;
    }

    private boolean runTrigger() {
        return !ballAtTrigger() && mPeriodicIO.ball_count > 0;
    }

    public void setForceTunnel(boolean enable) {
        mPeriodicIO.forceTunnelOn = enable;
    }

    private void updateBallCounter() {

        // reset count when we are outtaking for longer than 1 second
        if (mState == State.REVERSING) {
            if (!mWasReversing) {
                mEmptyIndexerTimer.start();
                mWasReversing = true;
            } else if (mEmptyIndexerTimer.hasElapsed(1.0)) {
                mPeriodicIO.ball_count = 0;
                mWasReversing = false;
                mEmptyIndexerTimer.reset();
            }
        } else {
            mWasReversing = false;
            mEmptyIndexerTimer.reset();

            // bottom beam break counts up when we index 
            if (mPeriodicIO.bottom_break) {
                if (!mBottomHadSeenBall) {
                    mPeriodicIO.ball_count++;
                    mBottomHadSeenBall = true;
                }
            } else {
                if (mBottomHadSeenBall) {
                    mBottomHadSeenBall = false;
                }
            }
        }
        // top beam break counts down when we shoot
        if (mPeriodicIO.top_break) {
            if (!mTopHadSeenBall) {
                mTopHadSeenBall = true;
            }
        } else {
            if (mTopHadSeenBall) {
                if (mState == State.FEEDING) {
                    mPeriodicIO.ball_count--;
                }
                mTopHadSeenBall = false;
            }
        }
    }

    private void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kIdleVoltage;
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kIdleVoltage;
                break;
            case INDEXING:
                if (runTrigger()) {
                    mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerIndexingVoltage;
                } else { 
                    mPeriodicIO.trigger_demand = 0.0;
                }

                if (stopTunnel()) {
                    mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kIdleVoltage;
                } else {
                    mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kTunnelIndexingVoltage;
                }
                break;
            case FEEDING:
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kFeedingVoltage;
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kFeedingVoltage;
                break;
            case REVERSING:
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerReversingVoltage;
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kTunnelReversingVoltage;
                break;
        }
    }
    
    @Override
    public void stop() {
        //mTrigger.set(ControlMode.PercentOutput, 0);
        mTunnel.set(ControlMode.PercentOutput, 0);

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
        public double tunnel_velocity;

        public double trigger_voltage;
        public double trigger_current;
        public double trigger_velocity;

        public boolean top_break;
        public boolean bottom_break;
        public boolean correctColor;
        public double ball_count;

        public boolean forceTunnelOn;

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
