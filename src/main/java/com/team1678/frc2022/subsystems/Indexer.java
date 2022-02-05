package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Indexer extends Subsystem {
    
private static Indexer mInstance;
public PeriodicIO mPeriodicIO = new PeriodicIO();

private TalonFX mOuttake;
private TalonFX mIndexer;
private TalonFX mTrigger;

private final DigitalInput mBottomBeamBreak;
private final DigitalInput mTopBeamBreak;

private boolean mRunTrigger = false;

private State mState = State.IDLE;

public enum WantedAction {
    NONE, INDEX, OUTTAKE, REVERSE, 
}

public enum State{
    IDLE, INDEXING, OUTTAKING, REVERSING,
}

private Indexer() {
    mOuttake = TalonFXFactory.createDefaultTalon(Ports.OUTTAKE_ID);
    mIndexer = TalonFXFactory.createDefaultTalon(Ports.INDEXER_ID);
    mTrigger = TalonFXFactory.createDefaultTalon(Ports.TRIGGER_ID);
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
            case OUTTAKE:
                mState = State.OUTTAKING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
        }
    
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.Outtake_demand = Constants.IndexerConstants.kIndexerIdleVoltage;
                mPeriodicIO.Indexer_demand = Constants.IndexerConstants.kIndexerIdleVoltage;
                mPeriodicIO.Trigger_demand = Constants.IndexerConstants.kIndexerIdleVoltage;
                break;
            case INDEXING:
                if (mRunTrigger) {
                    
                }
                mPeriodicIO.Indexer_demand = Constants.IndexerConstants.kIndexerIndexingVoltage;
                if (mPeriodicIO.bottomLightBeamBreakSensor) {
                    if (mPeriodicIO.correctColor) {
                        mPeriodicIO.eject = false;
                        if (mPeriodicIO.topLightBeamBreakSensor) {
                            mState = State.INDEXING;
                        } else {
                            mState = State.INDEXING;
                        }
                    } else {
                        mPeriodicIO.eject = false;
                    }
                } else {
                    if (mPeriodicIO.topLightBeamBreakSensor) {/
                        mState = State.INDEXING;
                    } else {
                        mState = State.INDEXING;
                    }
                }
                break;
            case OUTTAKING:
                mPeriodicIO.demand = Constants.IndexerConstants.kIndexerOuttakingVoltage;
                break;
            case REVERSING:
                mPeriodicIO.demand = Constants.IndexerConstants.kIndexerReversingVoltage;
                break;

        }
    }
    
    
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
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
            }
            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stop();
            }
        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mOuttake.set(ControlMode.PercentOutput, mPeriodicIO.Outtake_demand/12.0);
        mIndexer.set(ControlMode.PercentOutput, mPeriodicIO.Indexer_demand/12.0);  
        mTrigger.set(ControlMode.PercentOutput, mPeriodicIO.Trigger_demand/12.0); 
    }


    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.topLightBeamBreakSensor = mBottomBeamBreak.get();
        mPeriodicIO.bottomLightBeamBreakSensor = mTopBeamBreak.get();
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

    public static Indexer getInstance() {
        return null;
    }
    
    public static class PeriodicIO {
        // INPUTS
        public boolean topLightBeamBreakSensor;
        public boolean bottomLightBeamBreakSensor;
        public boolean correctColor;
        public Color detected_color;
        
        public double Outtake_current;
        public double Indexer_current;
        public double Trigger_current;
        
        public double Outtake_voltage;
        public double Indexer_voltage;
        public double Trigger_voltage;

        // OUTPUTS
        public double Outtake_demand;
        public double Indexer_demand;
        public double Trigger_demand;

        public boolean eject;
    }
}
   

    