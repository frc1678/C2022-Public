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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {

    public enum WantedAction {
        NONE, INTAKE, REVERSE
    }

    public enum State {
        IDLE, INTAKING, REVERSING
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
        mDeploy.setInverted(true);
        mSingulator.setInverted(true);

        mDeploy.setNeutralMode(NeutralMode.Coast);
        
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
        mPeriodicIO.intake_current = mRoller.getStatorCurrent();
        mPeriodicIO.intake_voltage = mRoller.getMotorOutputVoltage();

        mPeriodicIO.deploy_current = mDeploy.getStatorCurrent();
        mPeriodicIO.deploy_voltage = mDeploy.getMotorOutputVoltage();
        
        if (mPeriodicIO.deploy_current > Constants.IntakeConstants.kDeployCurrentLimit) {
            mPeriodicIO.hold_intake = true;
        }

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }

        SmartDashboard.putBoolean("Hold Intake", mPeriodicIO.hold_intake);
        SmartDashboard.putNumber("Deploy Demand", mPeriodicIO.deploy_demand);
        SmartDashboard.putNumber("Deploy Voltage", mPeriodicIO.deploy_voltage);
        SmartDashboard.putNumber("Deploy Current", mPeriodicIO.deploy_current);
        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        mRoller.set(ControlMode.PercentOutput, mPeriodicIO.intake_demand / 12.0);
        mSingulator.set(ControlMode.PercentOutput, mPeriodicIO.singulator_demand / 12.0);
        mDeploy.set(ControlMode.PercentOutput, mPeriodicIO.deploy_demand / 12.0);
    }
    
    public void setState(State state) {
        if (this.mState != state) {
            mPeriodicIO.hold_intake = false;
        }

        this.mState = state;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                if (mState != State.IDLE) {
                    mPeriodicIO.hold_intake = false;
                    mState = State.IDLE;                    
                }
                break;
            case INTAKE:
                if (mState != State.INTAKING) {
                    mPeriodicIO.hold_intake = false;
                    mState = State.INTAKING;
                }
                break;
            case REVERSE:
                if (mState != State.REVERSING) {
                    mPeriodicIO.hold_intake = false;
                    mState = State.REVERSING;
                }
                break;
        }
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.intake_demand = 0;
                mPeriodicIO.singulator_demand = 0;

                if (mPeriodicIO.hold_intake) {
                    mPeriodicIO.deploy_demand = -Constants.IntakeConstants.kHoldingVoltage;
                } else {
                    mPeriodicIO.deploy_demand = -Constants.IntakeConstants.kDeployVoltage;
                }
                break;
            case INTAKING:
                mPeriodicIO.intake_demand = Constants.IntakeConstants.kIntakingVoltage;
                mPeriodicIO.singulator_demand = Constants.IntakeConstants.kSingulatorVoltage;

                if (mPeriodicIO.hold_intake) {
                    mPeriodicIO.deploy_demand = Constants.IntakeConstants.kHoldingVoltage;
                } else {
                    mPeriodicIO.deploy_demand = Constants.IntakeConstants.kDeployVoltage;
                }
                break;
            case REVERSING:
                mPeriodicIO.intake_demand = -Constants.IntakeConstants.kIntakingVoltage;
                mPeriodicIO.singulator_demand = -Constants.IntakeConstants.kSingulatorVoltage;
                
                if (mPeriodicIO.hold_intake) {
                    mPeriodicIO.deploy_demand = Constants.IntakeConstants.kHoldingVoltage;
                } else {
                    mPeriodicIO.deploy_demand = Constants.IntakeConstants.kDeployVoltage;
                }
                break;
        }
    }

    /* Subsystem Getters */
    public synchronized State getState() {
        return mState;
    }
    public double getRollerCurrent() {
        return mPeriodicIO.intake_current;
    }
    public double getRollerVoltage() {
        return mPeriodicIO.intake_voltage;
    }
    public double getRollerDemand() {
        return mPeriodicIO.intake_demand;
    }
    public double getDeployCurrent() {
        return mPeriodicIO.deploy_current;
    }
    public double getDeployVoltage() {
        return mPeriodicIO.deploy_voltage;
    }
    public double getDeployDemand() {
        return mPeriodicIO.deploy_demand;
    }
    public double getSingulatorCurrent() {
        return mPeriodicIO.intake_current;
    }
    public double getSingulatorVoltage() {
        return mPeriodicIO.singulator_voltage;
    }
    public double getSingulatorDemand() {
        return mPeriodicIO.singulator_demand;
    }

    public static class PeriodicIO {
        // INPUTS
        private double intake_current;
        private double singulator_current;
        private double deploy_current;

        private double intake_voltage;
        private double singulator_voltage;
        private double deploy_voltage;

        // OUTPUTS
        private double intake_demand;
        private double singulator_demand;
        private double deploy_demand;
        private boolean hold_intake;
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
        items.add(mPeriodicIO.intake_current);
        items.add(mPeriodicIO.singulator_current);
        items.add(mPeriodicIO.intake_voltage);
        items.add(mPeriodicIO.singulator_voltage);
        
        // add outputs
        items.add(mPeriodicIO.intake_demand);
        items.add(mPeriodicIO.singulator_demand);

        // send data to logging storage
        mStorage.addData(items);
    }

}
