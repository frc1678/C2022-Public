package com.team1678.frc2022.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
    LogStorage<PeriodicIO> mStorage = null;

    private State mState = State.IDLE;

    private final TalonFX mTrigger;

    private Trigger() {
        mTrigger = TalonFXFactory.createDefaultTalon(Ports.TRIGGER_ID);
        mTrigger.setInverted(false);

        mTrigger.config_kP(0, Constants.TriggerConstants.kTriggerP, Constants.kLongCANTimeoutMs);
        mTrigger.config_kI(0, Constants.TriggerConstants.kTriggerI, Constants.kLongCANTimeoutMs);
        mTrigger.config_kD(0, Constants.TriggerConstants.kTriggerD, Constants.kLongCANTimeoutMs);
        mTrigger.config_kF(0, Constants.TriggerConstants.kTriggerF, Constants.kLongCANTimeoutMs);
        mTrigger.config_IntegralZone(0, (int) (200.0 / Constants.ShooterConstants.kFlywheelVelocityConversion));
        mTrigger.selectProfileSlot(0, 0);
        mTrigger.configClosedloopRamp(0.1);

        mTrigger.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
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
            }

            @Override
            public void onLoop(double timestamp) {
                runStateMachine();
                SendLog();
            }

            @Override
            public void onStop(double timestamp) {
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
                mPeriodicIO.demand = Constants.TriggerConstants.kTriggerPassiveVelocity;
                break;
            case FEEDING:
                mPeriodicIO.demand = Constants.TriggerConstants.kTriggerFeedingVelocity;
                break;
            case SLOW_FEEDING:
                mPeriodicIO.demand = Constants.TriggerConstants.kTriggerSlowFeedVelocity;
                break;
            case REVERSING:
                mPeriodicIO.demand = Constants.TriggerConstants.kTriggerReverseVelocity;
                break;
        }
    }
    
    public static class PeriodicIO {
        // INPUTS
        public double current;
        public double voltage;
        public double velocity;
        
        // OUTPUT
        public double demand;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.current = mTrigger.getStatorCurrent();
        mPeriodicIO.voltage = mTrigger.getMotorOutputVoltage();
        mPeriodicIO.velocity = mTrigger.getSelectedSensorVelocity() * Constants.TriggerConstants.kTriggerVelocityConversion;
        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        mTrigger.set(ControlMode.Velocity, mPeriodicIO.demand / Constants.TriggerConstants.kTriggerVelocityConversion);
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

    public double getTriggerVelocity() {
        return mPeriodicIO.velocity;
    }


    public String getTriggerState() {
        return mState.toString();
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
        LS.register(mStorage, "TRIGGER-LOGS.csv");
    }
    
    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");

        headers.add("trigger_demand");
        headers.add("trigger_voltage");
        headers.add("trigger_current");
        
        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());

        items.add(mPeriodicIO.demand);
        items.add(mPeriodicIO.voltage);
        items.add(mPeriodicIO.current);
        
        // send data to logging storage
        mStorage.addData(items);
    }
}
