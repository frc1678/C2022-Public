package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends Subsystem {

    private TimeDelayedBoolean mIntakeSolenoidTimer = new TimeDelayedBoolean();

    public enum WantedAction {
        NONE, INTAKE, REVERSE, STAY_OUT
    }

    public enum State {
        IDLE, INTAKING, REVERSING, STAYING_OUT
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private static Intake mInstance;
    public State mState = State.IDLE;

    private final TalonFX mMaster;
    private final TalonFX mSingulator;
    private Solenoid mSolenoid;

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ID);
        mSingulator = TalonFXFactory.createDefaultTalon(Ports.SINGULATOR_ID);
        mSolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.INTAKE_SOLENOID_ID);
    }

    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public void setState(State state) {
        this.mState = state;
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    public void zeroSensors() {
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.intake_demand = 0;
                mPeriodicIO.singulator_demand = 0;
                mPeriodicIO.deploy = false;
                break;
            case INTAKING:
                mPeriodicIO.intake_demand = Constants.IntakeConstants.kIntakingVoltage;
                mPeriodicIO.singulator_demand = Constants.IndexerConstants.kSingulatorVoltage;
                mPeriodicIO.deploy = true;
                break;
            case REVERSING:
                mPeriodicIO.intake_demand = -Constants.IntakeConstants.kIntakingVoltage;
                mPeriodicIO.singulator_demand = -Constants.IndexerConstants.kSingulatorVoltage;
                mPeriodicIO.deploy = true;
            case STAYING_OUT:
                mPeriodicIO.intake_demand = 0;
                mPeriodicIO.deploy = true;
                break;
        }
    }

    public synchronized State getState() {
        return mState;
    }

    // sets states
    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INTAKE:
                mState = State.INTAKING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
            case STAY_OUT:
                mState = State.STAYING_OUT;
                break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.intake_out = mIntakeSolenoidTimer.update(mPeriodicIO.deploy, 0.2);
        mPeriodicIO.intake_current = mMaster.getStatorCurrent();
        mPeriodicIO.intake_voltage = mMaster.getMotorOutputVoltage();
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.intake_demand / 12.0);
        mSingulator.set(ControlMode.PercentOutput, mPeriodicIO.singulator_demand / 12.0);
        mSolenoid.set(mPeriodicIO.deploy);
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
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean intake_out;
        public double intake_current;
        public double singulator_current;
        public double intake_voltage;
        public double singulator_voltage;

        // OUTPUTS
        public double intake_demand;
        public double singulator_demand;
        public boolean deploy;
    }

    public boolean getIsDeployed() {
        return mPeriodicIO.intake_out;
    }

    public boolean getWantDeploy() {
        return mPeriodicIO.deploy;
    }

    public double getIntakeVoltage() {
        return mPeriodicIO.intake_voltage;
    }

    public double getSingulatorVoltage() {
        return mPeriodicIO.singulator_voltage;
    }

    public double getIntakeCurrent() {
        return mPeriodicIO.intake_current;
    }

    public double getSingulatorCurrent() {
        return mPeriodicIO.intake_current;
    }

    public double getIntakeDemand() {
        return mPeriodicIO.intake_demand;
    }

    public double getSingulatorDemand() {
        return mPeriodicIO.singulator_demand;
    }
}
