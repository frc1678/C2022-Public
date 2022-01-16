package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.Ports;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Climber extends Subsystem {

    private TalonFX mClimber;
    private final Solenoid mSolenoid;

    private static Climber mInstance;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private Climber() {
        mSolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.CLIMBER_PIVOT_SOLENOID);
        mClimber = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_ID);
    }

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public void setState (State state) {
        this.mState = state;
    }

    public enum WantedAction {
        NONE, EXTEND, CLIMB, DEPLOY, RETRACT
    }

    public enum State {
        IDLE, EXTENDING, CLIMBING, DEPLOYING, RETRACTING
    }

    public State mState = State.IDLE;

    @Override
    public void writePeriodicOutputs() {
        mClimber.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand / 12.0);
        mSolenoid.set(mPeriodicIO.climber_solenoid);
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
              synchronized (Climber.this) {
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

    public synchronized State getState() {
        return mState;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case EXTEND:
                mState = State.EXTENDING;
                break;
            case CLIMB:
                mState = State.CLIMBING;
                break;
            case DEPLOY:
                mState = State.DEPLOYING;
                break;
            case RETRACT:
                mState = State.RETRACTING;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.climber_voltage = mClimber.getMotorOutputVoltage();
        mPeriodicIO.climber_current = mClimber.getStatorCurrent();
        mPeriodicIO.climber_solenoid = mSolenoid.get();
    }

    private void runStateMachine() {
        switch (mState) {
            case EXTENDING:
                mPeriodicIO.climber_demand = Constants.ClimberConstants.kExtendingVoltage;
                break;
            case CLIMBING:
                mPeriodicIO.climber_demand = Constants.ClimberConstants.kRetractingVoltage;
                break;
            case DEPLOYING:
                mPeriodicIO.climber_solenoid = true;
            case RETRACTING:
                mPeriodicIO.climber_solenoid = false;
            case IDLE:
                mPeriodicIO.climber_demand = Constants.ClimberConstants.kIdleVoltage;
                break;
        }
    }

    public boolean checkSystem() {
        return true;
    }

    @Override
    public void stop(){
        setState(WantedAction.NONE);
        mClimber.set(ControlMode.PercentOutput, 0);
    }

    public double getMotorVoltage() {
        return mPeriodicIO.climber_voltage;
    }

    public double getMotorCurrent() {
        return mPeriodicIO.climber_current;  
    }

    public boolean getSolenoid() {
        return mPeriodicIO.climber_solenoid;
    }

    public static class PeriodicIO {
        //INPUTS
        public double climber_voltage;
        public double climber_current;
        public boolean climber_solenoid;

        //OUTPUTS
        public double climber_demand;
    }
}   
