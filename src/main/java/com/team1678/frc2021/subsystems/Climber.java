package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.Ports;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {

    private TalonFX mClimber;
    private final Solenoid mShiftSolenoid;

    private static Climber mInstance;

    private TimeDelayedBoolean mShiftSolenoidTimer = new TimeDelayedBoolean();
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private Climber() {
        mShiftSolenoid = null;
        mClimber = new TalonFX(Ports.HOPPER_ID);
    }

    public enum WantedAction {
        NONE, EXTEND, RETRACT,
    }

    public enum State {
        IDLE, EXTENDING, RETRACTING,
    }

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 80, 80, 1.0);
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private State mState = State.IDLE;

    @Override
    public void writePeriodicOutputs() {
        mClimber.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand / 12.0);
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

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case EXTEND:
                mState = State.EXTENDING;
                break;
            case RETRACT:
                mState = State.RETRACTING;
                break;
        }
    }

    private void runStateMachine() {
        switch (mState) {
            case EXTENDING:
                mPeriodicIO.climber_demand = Constants.ClimberConstants.kExtendingVoltage;
                break;
            case RETRACTING:
                mPeriodicIO.climber_demand = Constants.ClimberConstants.kRetractingVoltage;
                break;
            case IDLE:
                mPeriodicIO.climber_demand = Constants.ClimberConstants.kIdleVoltage;
                break;
        }
    }

    public boolean checkSystem() {
        return true;
    }

    public boolean hasEmergency = false;

    public void outputTelemetry(){
        SmartDashboard.putNumber("Climber Voltage", mPeriodicIO.climber_voltage);
        SmartDashboard.putNumber("Climber Current", mPeriodicIO.climber_current);
        SmartDashboard.putNumber("Climber Demand", mPeriodicIO. climber_demand);
        SmartDashboard.putString("Climber State", mState.toString());
    }

    @Override
    public void stop(){
        setState(WantedAction.NONE);
        mClimber.set(ControlMode.PercentOutput, 0);
    }

    public static class PeriodicIO {
        //INPUTS
        public double climber_voltage;
        public double climber_current;

        //OUTPUTS
        public double climber_demand;
    }
}   
