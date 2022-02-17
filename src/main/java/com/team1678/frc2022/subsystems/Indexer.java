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
import edu.wpi.first.wpilibj.util.Color;

public class Indexer extends Subsystem {
    
    private static Indexer mInstance;
    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private TalonFX mEjector;
    private TalonFX mTunnel;
    private TalonFX mTrigger;

    private final DigitalInput mBottomBeamBreak;
    private final DigitalInput mTopBeamBreak;

    public boolean mBottomHadSeenBall = false;
    public boolean mTopHadSeenBall = false;

    private final ColorSensor mColorSensor;

    private boolean mRunTrigger() {
        return !mBallAtTrigger() && mPeriodicIO.ball_count > 0;
    }

    private boolean mBallAtTrigger() {
        return mPeriodicIO.topLightBeamBreakSensor;
    }

    private boolean stopIndexer() {
        if (!mBallInIndexer() && mPeriodicIO.ball_count <= 1) {
            return false;
        }
        else {
            return true;
        }
    }

    private boolean mBallInIndexer () {
        return mPeriodicIO.bottomLightBeamBreakSensor;
    }

    private State mState = State.IDLE;

    public enum WantedAction {
        NONE, INDEX, EJECT, REVERSE, 
    }

    public enum State{
        IDLE, INDEXING, EJECTING, REVERSING,
    }

    private Indexer() {
        mEjector = TalonFXFactory.createDefaultTalon(Ports.OUTTAKE_ID);
        mTunnel = TalonFXFactory.createDefaultTalon(Ports.TUNNEL_ID);
        mTrigger = TalonFXFactory.createDefaultTalon(Ports.TRIGGER_ID);
        mBottomBeamBreak = new DigitalInput(Ports.BOTTOM_BEAM_BREAK);
        mTopBeamBreak = new DigitalInput(Ports.TOP_BEAM_BREAK);
        mColorSensor =  ColorSensor.getInstance();
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
            case REVERSE:
                mState = State.REVERSING;
                break;
        }
    
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.ejector_demand = Constants.IndexerConstants.kOuttakeIdleVoltage;
                mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerIdleVoltage;
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerIdleVoltage;
                break;
            case INDEXING:
                // Check for whether we want to outtake based off color sensor
                if (mColorSensor.getOuttake()) {
                    // if not correct color, goes to EJECTING state
                    this.setState(WantedAction.EJECT);
                } else {
                    // runs trigger if top beam break isn't triggered
                    if (mRunTrigger()) {
                        mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerIndexingVoltage;
                    } else {
                        mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerIdleVoltage;
                    }
                    // stops indexer if bottom beam break is triggered, otherwise keeps indexing
                    if (stopIndexer()) {
                        mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerIdleVoltage;
                    } else {
                        mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerIndexingVoltage;
                    }

                    mPeriodicIO.ejector_demand = Constants.IndexerConstants.kOuttakeIdleVoltage;   
                }
                break;
            case EJECTING:
                //if not the correct color, outtakes the ball
                if (mColorSensor.getOuttake()) {
                    mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerReversingVoltage;
                    mPeriodicIO.ejector_demand = Constants.IndexerConstants.kOuttakeReversingVoltage;
                } else if (!mColorSensor.getOuttake()) {
                    //if it is the corect color, it goes to the indexing state
                    this.setState(WantedAction.INDEX);
                } 
                break;
            case REVERSING:
                // reverses everything
                mPeriodicIO.ejector_demand = Constants.IndexerConstants.kOuttakeIndexingVoltage;
                mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerReversingVoltage;
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerReversingVoltage;
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
   

    @Override
    public synchronized void writePeriodicOutputs() {
        mEjector.set(ControlMode.PercentOutput, mPeriodicIO.ejector_demand/12.0);
        mTunnel.set(ControlMode.PercentOutput, mPeriodicIO.indexer_demand/12.0);  
        mTrigger.set(ControlMode.PercentOutput, mPeriodicIO.trigger_demand/12.0); 
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.topLightBeamBreakSensor = mBottomBeamBreak.get();
        mPeriodicIO.bottomLightBeamBreakSensor = mTopBeamBreak.get();

        if (mPeriodicIO.bottomLightBeamBreakSensor) {
            if (!mBottomHadSeenBall) {
                mBottomHadSeenBall = true;
            }
        } else {
            if (mBottomHadSeenBall) {
                mPeriodicIO.ball_count++;
                mBottomHadSeenBall = false;
            }
        }

        if (mPeriodicIO.topLightBeamBreakSensor) {
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

    public Object getState() {
        return null;
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
        mPeriodicIO.indexer_demand = demand;
    }

    //subsystem getters
    public static Indexer getInstance() {
        return null;
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
        return mPeriodicIO.indexer_demand;
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
        return mPeriodicIO.topLightBeamBreakSensor;
    }

    public boolean getBottomBeamBreak() {
        return mPeriodicIO.bottomLightBeamBreakSensor;
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean topLightBeamBreakSensor;
        public boolean bottomLightBeamBreakSensor;
        public double ball_count;
        
        public double ejector_current;
        public double tunnel_current;
        public double trigger_current;
        
        public double ejector_voltage;
        public double tunnel_voltage;
        public double trigger_voltage;

        public Color detected_color;

        // OUTPUTS
        public double ejector_demand;
        public double indexer_demand;
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
   

    