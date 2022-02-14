package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
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

public class Intake extends Subsystem {

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
    private final TalonFX mDeploy;

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ROLLER_ID);
        mSingulator = TalonFXFactory.createDefaultTalon(Ports.SINGULATOR_ID);
        mDeploy = TalonFXFactory.createDefaultTalon(Ports.INTAKE_DEPLOY_ID);

        mDeploy.config_kP(0, Constants.IntakeConstants.kDeployP, Constants.kLongCANTimeoutMs);
        mDeploy.config_kI(0, Constants.IntakeConstants.kDeployI, Constants.kLongCANTimeoutMs);
        mDeploy.config_kD(0, Constants.IntakeConstants.kDeployD, Constants.kLongCANTimeoutMs);
        mDeploy.config_kF(0, Constants.IntakeConstants.kDeployF, Constants.kLongCANTimeoutMs);

        // reduce can util
        mMaster.changeMotionControlFramePeriod(255);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        mSingulator.changeMotionControlFramePeriod(255);
        mSingulator.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mSingulator.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        mDeploy.changeMotionControlFramePeriod(255);
        mDeploy.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mDeploy.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        
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
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.deploy_voltage = mDeploy.getMotorOutputVoltage();
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
        mDeploy.set(ControlMode.MotionMagic, mPeriodicIO.deploy_demand);
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
                mPeriodicIO.deploy_demand = Constants.IntakeConstants.kDeployIdlePosition;
                break;
            case INTAKING:
                mPeriodicIO.intake_demand = Constants.IntakeConstants.kIntakingVoltage;
                mPeriodicIO.singulator_demand = Constants.IndexerConstants.kSingulatorVoltage;
                mPeriodicIO.deploy_demand = Constants.IntakeConstants.kDeployExtendPosition;
                break;
            case REVERSING:
                mPeriodicIO.intake_demand = -Constants.IntakeConstants.kIntakingVoltage;
                mPeriodicIO.singulator_demand = -Constants.IndexerConstants.kSingulatorVoltage;
                mPeriodicIO.deploy_demand = Constants.IntakeConstants.kDeployExtendPosition;
                break;
            case STAYING_OUT:
                mPeriodicIO.intake_demand = 0;
                mPeriodicIO.deploy_demand = Constants.IntakeConstants.kDeployExtendPosition;
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
        return mPeriodicIO.deploy_demand == Constants.IntakeConstants.kDeployIdlePosition;
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
        private double intake_voltage;
        private double singulator_voltage;
        private double singulator_current;
        private double deploy_voltage;

        // OUTPUTS
        private double intake_demand;
        private double singulator_demand;
        private double deploy_demand;
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public void zeroSensors() {
        // empty
    }
    
}
