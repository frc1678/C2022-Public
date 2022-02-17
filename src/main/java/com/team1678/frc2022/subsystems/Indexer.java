package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {
    
    private static Indexer mInstance;
    public static synchronized Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private TalonFX mEjector;
    private TalonFX mTunnel;
    private TalonFX mTrigger;

    private final DigitalInput mBottomBeamBreak;
    private final DigitalInput mTopBeamBreak;

    private boolean mBottomHadSeenBall = false;
    private boolean mTopHadSeenBall = false;

    private boolean stopTunnel() {
        return ballAtTrigger() && ballInTunnel() /*|| mPeriodicIO.ball_count == 2*/;
    }

    private boolean ballAtTrigger() {
        return mPeriodicIO.top_break;
    }

    private boolean ballInTunnel () {
        return mPeriodicIO.bottom_break;
    }

    private State mState = State.IDLE;

    public enum WantedAction {
        NONE, INDEX, EJECT, FEED, REVERSE, 
    }

    public enum State{
        IDLE, INDEXING, EJECTING, FEEDING, REVERSING,
    }

    private Indexer() {
        mEjector = TalonFXFactory.createDefaultTalon(Ports.EJECTOR_ID);
        mTunnel = TalonFXFactory.createDefaultTalon(Ports.TUNNEL_ID);
        mTrigger = TalonFXFactory.createDefaultTalon(Ports.TRIGGER_ID);

        mTunnel.setInverted(true);
        mTrigger.setInverted(true);

        mBottomBeamBreak = new DigitalInput(Ports.BOTTOM_BEAM_BREAK);
        mTopBeamBreak = new DigitalInput(Ports.TOP_BEAM_BREAK);
    }
            
    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INDEX:
                mState = State.INDEXING;
                break;
            case EJECT:
                mState = State.EJECTING;
                break;
            case FEED:
                mState = State.FEEDING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
        }
    
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.ejector_demand = Constants.IndexerConstants.kIdleVoltage;
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kIdleVoltage;
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kIdleVoltage;
                break;
            case INDEXING:
                if (!stopTunnel()) {
                    mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kTunnelVoltage;
                    mPeriodicIO.ejector_demand = Constants.IndexerConstants.kEjectorVoltage;
                } else {
                    mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kIdleVoltage;
                    mPeriodicIO.ejector_demand = Constants.IndexerConstants.kIdleVoltage;
                }
                mPeriodicIO.trigger_demand = -0.5;
                break;
            case EJECTING:
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kTunnelVoltage;
                mPeriodicIO.ejector_demand = -Constants.IndexerConstants.kEjectorVoltage;
                mPeriodicIO.trigger_demand = -0.5;
                break;
            case FEEDING:
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kFeedingVoltage;
                mPeriodicIO.ejector_demand = Constants.IndexerConstants.kEjectorVoltage;
                mPeriodicIO.trigger_demand = -Constants.IndexerConstants.kFeedingVoltage;
                break;
            case REVERSING:
                // reverses everything
                mPeriodicIO.ejector_demand = Constants.IndexerConstants.kIdleVoltage;
                mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kReversingVoltage;
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kReversingVoltage;
                break;
        }
    }
    
    @Override
    public void registerEnabledLoops (ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
                startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Indexer.this){
                    runStateMachine();
                    updateBallCounter();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stopLogging();
                stop();
            }
        });
    }

    private void updateBallCounter() {

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
   
    @Override
    public synchronized void writePeriodicOutputs() {
        mEjector.set(ControlMode.PercentOutput, mPeriodicIO.ejector_demand / 12.0);
        mTunnel.set(ControlMode.PercentOutput, mPeriodicIO.tunnel_demand / 12.0);  
        mTrigger.set(ControlMode.PercentOutput, mPeriodicIO.trigger_demand / 12.0);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.top_break = !mTopBeamBreak.get();
        mPeriodicIO.bottom_break = !mBottomBeamBreak.get();
    }

    public State getState() {
        return mState;
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    } 

    //subsystem setters
    public void setTriggerDemand(double demand) {
        mPeriodicIO.trigger_demand = demand;
    }
    
    public void setOuttakeDemand(double demand) {
        mPeriodicIO.ejector_demand = demand;
    }

    public void setIndexerDemand(double demand) {
        mPeriodicIO.tunnel_demand = demand;
    }

    public double getEjectorDemand() {
       return mPeriodicIO.ejector_demand;
    }

    public double getEjectorCurrent() {
        return mPeriodicIO.ejector_current;
    }

    public double getEjectorVoltage() {
        return mPeriodicIO.ejector_voltage;
    }

    public double getTunnelCurrent() {
        return mPeriodicIO.tunnel_current;
    }

    public double getTunnelDemand() {
        return mPeriodicIO.tunnel_demand;
    }

    public double getTunnelVoltage() {
        return mPeriodicIO.tunnel_voltage;
    }

    public double getTriggerCurrent() {
        return mPeriodicIO.trigger_current;
    }

    public double getTriggerDemand() {
        return mPeriodicIO.trigger_demand;
    }

    public double getTriggerVoltage() {
        return mPeriodicIO.trigger_voltage;
    }

    public double getBallCount() {
        return mPeriodicIO.ball_count;
    }

    public boolean getTopBeamBreak() {
        return mPeriodicIO.top_break;
    }

    public boolean getBottomBeamBreak() {
        return mPeriodicIO.bottom_break;
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean top_break;
        public boolean bottom_break;
        public double ball_count;
        
        public double ejector_current;
        public double tunnel_current;
        public double trigger_current;
        
        public double ejector_voltage;
        public double tunnel_voltage;
        public double trigger_voltage;

        // OUTPUTS
        public double ejector_demand;
        public double tunnel_demand;
        public double trigger_demand;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/INDEXER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
    // only call for quick status testing
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Top Had Seen Ball", mTopHadSeenBall);
        SmartDashboard.putBoolean("Bottom Had Seen Ball", mBottomHadSeenBall);
    }
    
}
   

    