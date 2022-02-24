package com.team1678.frc2022.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.Timer;

public class Trigger extends Subsystem {

    public enum WantedAction {
        NONE, PASSIVE_REVERSE, FEED, SLOW_FEED, REVERSE
    }

    public enum State {
        IDLE, PASSIVE_REVERSING, FEEDING, SLOW_FEEDING, REVERSING
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    LogStorage<PeriodicIO> mStorage = null;

    private State mState = State.IDLE;

    private final TalonFX mTrigger;

    private Trigger() {
        mTrigger = TalonFXFactory.createDefaultTalon(Ports.TRIGGER_ID);
        mTrigger.setInverted(true);
    }

    private static Trigger mInstance;
    
    public static Trigger getInstance() {
        if (mInstance == null) {
            mInstance = new Trigger();
        }
        return mInstance;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
                startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                runStateMachine();
            }

            @Override
            public void onStop(double timestamp) {
                stopLogging();
            }
        });
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case PASSIVE_REVERSE:
                mState = State.PASSIVE_REVERSING;
                break;
            case FEED:
                mState = State.FEEDING;
                break;
            case SLOW_FEED:
                mState = State.SLOW_FEEDING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
        }
    }

    private void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = 0.0;
                break;
            case PASSIVE_REVERSING:
                mPeriodicIO.demand = Constants.TriggerConstants.kTriggerPassiveVoltage;
                break;
            case FEEDING:
                mPeriodicIO.demand = Constants.TriggerConstants.kTriggerFeedingVoltage;
                break;
            case SLOW_FEEDING:
                mPeriodicIO.demand = Constants.TriggerConstants.kTriggerSlowFeedVoltage;
                break;
            case REVERSING:
                mPeriodicIO.demand = Constants.TriggerConstants.kTriggerReverseVoltage;
                break;
        }
    }
    
    public static class PeriodicIO {
        // INPUTS
        public double current;
        public double voltage;
        
        // OUTPUT
        public double demand;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.current = mTrigger.getStatorCurrent();
        mPeriodicIO.voltage = mTrigger.getMotorOutputVoltage();
        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        mTrigger.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
    }

    public double getTriggerCurrent() {
        return mPeriodicIO.current;
    }

    public double getTriggerVoltage() {
        return mPeriodicIO.voltage;
    }

    public double getTriggerDemand() {
        return mPeriodicIO.demand;
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

    // Logging
    
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "INTAKE_LOGS.csv");
    }
        
    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/TRIGGER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
    
    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();
        mStorage.setHeadersFromClass(PeriodicIO.class);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());

        // add inputs
        items.add(mPeriodicIO.current);
        items.add(mPeriodicIO.voltage);
        
        // add outputs
        items.add(mPeriodicIO.demand);
        items.add(mPeriodicIO.demand);

        // send data to logging storage
        mStorage.addData(items);
    }
}
