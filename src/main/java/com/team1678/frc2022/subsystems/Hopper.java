package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hopper extends Subsystem {


    private static Hopper mInstance;
    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    private State mState = State.SLOW_INDEXING;

    private TalonFX mHopper;

    public enum WantedAction {
        INDEX,
        REVERSE,
        SLOW_INDEX,
        NONE
    }

    public enum State {
        INDEXING,
        REVERSING,
        SLOW_INDEXING,
        IDLE,
    }

    private Hopper() {
        mHopper = new TalonFX(Ports.HOPPER_ID);
    }

    public static synchronized Hopper getInstance() {
        if (mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }

    public State getState() {
        return mState;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.hopper_current = mHopper.getStatorCurrent();
        mPeriodicIO.hopper_voltage = mHopper.getMotorOutputVoltage();
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.SLOW_INDEXING;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Hopper.this) {
                    runStateMachine();
                    outputTelemetry();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.SLOW_INDEXING;
                stop();
            }
        });
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case SLOW_INDEX:
                mState = State.SLOW_INDEXING;
                break;
            case INDEX:
                mState = State.INDEXING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
            case NONE:
                mState = State.IDLE;
                break;
        }

    }

    private void runStateMachine() {
        switch (mState) {
            case INDEXING:
                mPeriodicIO.hopper_demand = Constants.HopperConstants.kIntakingVoltage;
                break;
            case REVERSING:
                mPeriodicIO.hopper_demand = -Constants.HopperConstants.kIntakingVoltage;
                break;
            case SLOW_INDEXING:
                mPeriodicIO.hopper_demand = Constants.HopperConstants.kIdleVoltage;
                break;
            case IDLE:
                mPeriodicIO.hopper_demand = 0;
                break;
        }
    }

    @Override
    public void stop() {
        setState(WantedAction.SLOW_INDEX);
        mHopper.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Hopper State", mState.toString());
        SmartDashboard.putNumber("Hopper Demand", mPeriodicIO.hopper_demand);
        SmartDashboard.putNumber("Hopper Voltage", mPeriodicIO.hopper_voltage);
        SmartDashboard.putNumber("Hopper Current", mPeriodicIO.hopper_current);
    }

    @Override
    public void writePeriodicOutputs() {
        mHopper.set(ControlMode.PercentOutput, mPeriodicIO.hopper_demand / 12.0);
    }


    public static class PeriodicIO {
        // INPUTS
        public double hopper_voltage;
        public double hopper_current;

        // OUTPUTS
        public double hopper_demand;
    }
}