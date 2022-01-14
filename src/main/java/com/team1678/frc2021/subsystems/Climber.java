package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.subsystems.Subsystem;
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
        Constants.makeSolenoidForID(Ports.CLIMBER_PIVOT_SOLENOID);
        mClimber = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_ID);
    }

    public enum WantedAction {
        NONE, EXTEND, RETRACT, HOOK
    }

    public enum State {
        IDLE, EXTENDING, RETRACTING, HOOKING
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
            case EXTEND:
                mState = State.EXTENDING;
                break;
            case RETRACT:
                mState = State.RETRACTING;
                break;
            case HOOK:
                mState = State.HOOKING;
                break;
        }
    }

    private void runStateMachine() {
        switch (mState) {
            case EXTENDING:
                spinMotor(Constants.ClimberConstants.kExtendingVoltage);
                mPeriodicIO.climber_deploy_solenoid = false;
                break;
            case HOOKING:
                spinMotor(Constants.ClimberConstants.kExtendingVoltage);
                mPeriodicIO.climber_deploy_solenoid = true;
            case RETRACTING:
                spinMotor(Constants.ClimberConstants.kRetractingVoltage);
                mPeriodicIO.climber_deploy_solenoid = true;
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
        public boolean climber_deploy_solenoid;

        //OUTPUTS
        public double climber_demand;
    }
}   
