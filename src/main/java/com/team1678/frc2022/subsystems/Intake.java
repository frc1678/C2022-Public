package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Ports;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.ILooper;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

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

    private final TalonFX mRoller;
    private final TalonFX mDeploy;
    private final TalonFX mSingulator;

    private Intake() {
        mRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ROLLER_ID);
        mDeploy = TalonFXFactory.createDefaultTalon(Ports.INTAKE_DEPLOY_ID);
        mSingulator = TalonFXFactory.createDefaultTalon(Ports.SINGULATOR_ID);

        mRoller.setInverted(true);
        mSingulator.setInverted(true);

        mDeploy.setInverted(true);
        mDeploy.setNeutralMode(NeutralMode.Brake);
        mDeploy.setSelectedSensorPosition(0.0);
        mDeploy.config_kP(0, Constants.IntakeConstants.kIntakeP, Constants.kLongCANTimeoutMs);
        mDeploy.config_kI(0, Constants.IntakeConstants.kIntakeI, Constants.kLongCANTimeoutMs);
        mDeploy.config_kD(0, Constants.IntakeConstants.kIntakeD, Constants.kLongCANTimeoutMs);
        mDeploy.config_kF(0, Constants.IntakeConstants.kIntakeF, Constants.kLongCANTimeoutMs);
        
        SupplyCurrentLimitConfiguration main_curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 40, 0.02);
        mDeploy.configSupplyCurrentLimit(main_curr_lim);

        // reduce can util
        mRoller.changeMotionControlFramePeriod(255);
        mRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        mSingulator.changeMotionControlFramePeriod(255);
        mSingulator.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mSingulator.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
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
        mPeriodicIO.intake_current = mRoller.getStatorCurrent();
        mPeriodicIO.intake_voltage = mRoller.getMotorOutputVoltage();
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }

        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        mRoller.set(ControlMode.PercentOutput, mPeriodicIO.intake_demand / 12.0);
        mSingulator.set(ControlMode.PercentOutput, mPeriodicIO.singulator_demand / 12.0);

        /*
        if (mPeriodicIO.deploy) {
            mDeploy.set(ControlMode.Position, Constants.IntakeConstants.kDeployDelta);
        } else {
            mDeploy.set(ControlMode.Position, 10.0);
        }
        */
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
        mRoller.set(ControlMode.PercentOutput, 0);
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
        SetupLog();
        LS.register(mStorage, "INTAKE_LOGS.csv");
    }
    
    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();
        mStorage.setHeadersFromClass(PeriodicIO.class);
    }

    public void SendLog() {
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
