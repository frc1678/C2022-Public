package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Indexer extends Subsystem {
    
private static Indexer mInstance;
public PeriodicIO mPeriodicIO = new PeriodicIO();

private TalonFX mOuttake;
private TalonFX mIndexer;
private TalonFX mTrigger;

private State mState = State.IDLE;

private Indexer() {
    mOuttake = TalonFXFactory.createDefaultTalon(Ports.OUTTAKE_ID);
    mIndexer = TalonFXFactory.createDefaultTalon(Ports.INDEXER_ID);
    mTrigger = TalonFXFactory.createDefaultTalon(Ports.TRIGGER_ID);
}
        
    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
        }
    
    }

    
    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = Constants.IndexerConstants.kIndexerIdleVoltage;
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
        public void writePeriodicOutputs() {
            mOuttake.overrideSoftLimitsEnable(false);
            mOuttake.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
    
            mIndexer.overrideSoftLimitsEnable(false);
            mIndexer.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
    
            mTrigger.overrideSoftLimitsEnable(false);
            mTrigger.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        }
    
    public Object getState() {
        return null;
    }

    public enum WantedAction {
        NONE
    }

    public enum State {
        IDLE,
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

        // OUTPUTS
        public double demand;
    }
}
   

    