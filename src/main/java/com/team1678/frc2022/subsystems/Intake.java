package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Ports;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.ILooper;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

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

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    private static Intake mInstance;
    public State mState = State.IDLE;

    private final TalonFX mMaster;
    private final TalonFX mSingulator;
    private Solenoid mSolenoid;

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ID);
        mSingulator = TalonFXFactory.createDefaultTalon(Ports.SINGULATOR_ID);

        // reduce can util
        mMaster.changeMotionControlFramePeriod(255);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        mSingulator.changeMotionControlFramePeriod(255);
        mSingulator.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mSingulator.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);

        mSolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.INTAKE_SOLENOID_ID);
    }

    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
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
    
    public void setState(State state) {
        this.mState = state;
    }

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
                break;
            case STAYING_OUT:
                mPeriodicIO.intake_demand = 0;
                mPeriodicIO.deploy = true;
                break;
        }
    }

    /* Subsystem Getters */
    public synchronized State getState() {
        return mState;
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

    public static class PeriodicIO {
        // INPUTS
        private boolean intake_out;
        private double intake_current;
        private double singulator_current;
        private double intake_voltage;
        private double singulator_voltage;

        // OUTPUTS
        private double intake_demand;
        private double singulator_demand;
        private boolean deploy;
    }

    public void zeroSensors() {
        // empty
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
    
   public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/INTAKE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    // logger
    
    @Override
    public void registerLogger(LoggingSystem LS) {
        LogSetup();
        LS.register(mStorage, "INTAKE_LOGS.csv");
    }
    
    public void LogSetup() {
        mStorage = new LogStorage<PeriodicIO>();
        mStorage.setHeadersFromClass(PeriodicIO.class);
    }

    public void LogSend() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());

        // add inputs
        items.add(mPeriodicIO.intake_out ? 1.0 : 0.0);
        items.add(mPeriodicIO.intake_current);
        items.add(mPeriodicIO.singulator_current);
        items.add(mPeriodicIO.intake_voltage);
        items.add(mPeriodicIO.singulator_voltage);
        
        // add outputs
        items.add(mPeriodicIO.intake_demand);
        items.add(mPeriodicIO.singulator_demand);
        items.add(mPeriodicIO.deploy ? 1.0 : 0.0);

        // send data to logging storage
        mStorage.addData(items);
    }

}
