package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.Ports;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {

    private TalonFX mClimber;
    private final Solenoid mShiftSolenoid;

    private void spinMotor(double voltage){
        mPeriodicIO.climber_demand = voltage;
    }

    private static Climber mInstance;

    private TimeDelayedBoolean mShiftSolenoidTimer = new TimeDelayedBoolean();
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private Climber() {
        mShiftSolenoid = Solenoid(Ports.CLIMBER_PIVOT_SOLENOID);
        mClimber = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_ID);
    }

    private Solenoid Solenoid(int climberPivotSolenoid) {
        return null;
    }

    public static synchronized Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public enum WantedAction {
        NONE, GROUND_EXTEND, EXTEND, RETRACT, PIVOT
    }

    public enum State {
        IDLE, GROUND_EXTENDING, EXTENDING, RETRACTING, PIVOTING
    }

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
            case GROUND_EXTEND:
                mState = State.GROUND_EXTENDING;
            case EXTEND:
                mState = State.EXTENDING;
                break;
            case RETRACT:
                mState = State.RETRACTING;
                break;
            case PIVOT:
                mState = State.PIVOTING;
                break;
        }
    }

    private void runStateMachine() {
        switch (mState) {
            case EXTENDING:
                spinMotor(Constants.ClimberConstants.kExtendingVoltage);
                mPeriodicIO.climber_solenoid = true;
                break;
            case GROUND_EXTENDING:
                spinMotor(Constants.ClimberConstants.kExtendingVoltage);
                mPeriodicIO.climber_solenoid = false;
            case PIVOTING:
                spinMotor(Constants.ClimberConstants.kExtendingVoltage);
                mPeriodicIO.climber_solenoid = true;
            case RETRACTING:
                spinMotor(Constants.ClimberConstants.kRetractingVoltage);
                mPeriodicIO.climber_solenoid = true;
                break;
            case IDLE:
                spinMotor(Constants.ClimberConstants.kIdleVoltage);
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
        public boolean climber_solenoid;

        //OUTPUTS
        public double climber_demand;
    }
}   
