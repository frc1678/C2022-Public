package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {

    private TimeDelayedBoolean mClimberCalibrated = new TimeDelayedBoolean();
    private boolean mHomed = false;
    private boolean mIsHoming = false;
    private boolean mIsOpenLoop = false;

    /* Subsystem Instance */
    private static Climber mInstance;

    public static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }
    
    /* Components */
    private TalonFX mMaster;
    private TalonFX mSlave;
    private final Solenoid mDeploySolenoid;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 80, 80,
            1.0);

    private Climber() {
        mMaster = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_MASTER_ID);
        mSlave = TalonFXFactory.createPermanentSlaveTalon(Ports.CLIMBER_MASTER_ID, Ports.CLIMBER_SLAVE_ID);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mSlave.setInverted(true);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mMaster.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(0, 0.5);
        mMaster.config_kI(0, 0);
        mMaster.config_kD(0, 0);
        mMaster.config_kF(0, 0.05);

        mMaster.setNeutralMode(NeutralMode.Brake);
        mSlave.setNeutralMode(NeutralMode.Brake);

        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
        //mMaster.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);

        mDeploySolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.CLIMBER_PIVOT_SOLENOID);
    }

    private void zeroEncoder() {
        mMaster.setSelectedSensorPosition(0.0);
        mHomed = true;
        setClimberOpenLoop(0.0);
    }
    
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.stator_current = mMaster.getStatorCurrent();
        mPeriodicIO.motor_position = mMaster.getSelectedSensorPosition();
        mPeriodicIO.motor_velocity = mMaster.getSelectedSensorVelocity();

        SmartDashboard.putBoolean("Is homing", mIsHoming);

        if (!mHomed) {
            if (mPeriodicIO.stator_current > 30 & mIsHoming == false) {
                mIsHoming = true;
            }
            if (mClimberCalibrated.update(Util.epsilonEquals(mPeriodicIO.motor_velocity, 0, 500), Constants.ClimberConstants.kCalibrationTimeoutSeconds) && mIsHoming == true) {
                zeroEncoder();
            }
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (!mHomed) {
            mMaster.set(ControlMode.PercentOutput, Constants.ClimberConstants.kCalibratingVoltage);
        } else {
            if (!mIsOpenLoop) {
                mMaster.set(ControlMode.MotionMagic, mPeriodicIO.climber_demand);
            } else {
                mMaster.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand / 12);
            }
        }

        mDeploySolenoid.set(mPeriodicIO.deploy_solenoid);
    }

    public void setClimberOpenLoop(double wantedDemand) {
        if (mIsOpenLoop != true) {
            mIsOpenLoop = true;
        }
        mPeriodicIO.climber_demand = (wantedDemand > 8 ? 8 : wantedDemand);
    }

    public void setClimberPosition(double wantedPositionTicks) {
        if (mIsOpenLoop != false) {
            mIsOpenLoop = false;
        }
        mPeriodicIO.climber_demand = wantedPositionTicks;
    }

    public void setClimberPositionDelta(double wantedPositionDelta) {
        if (mIsOpenLoop != false) {
            mIsOpenLoop = false;
        }
        mPeriodicIO.climber_demand = mPeriodicIO.climber_demand + wantedPositionDelta;
    }

    public void setWantDeploy(boolean wantsDeploy) {
        mPeriodicIO.deploy_solenoid = wantsDeploy;
    }
    
    public boolean getHomed() {
        return mHomed;
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    public boolean checkSystem() {
        return true;
    }

    public double getStatorCurrent() {
        return mPeriodicIO.stator_current;
    }

    public double getMotorPosition() {
        return mPeriodicIO.motor_position;
    }

    public double getMotorVelocity() {
        return mPeriodicIO.motor_velocity;
    }

    public double getDemand() {
        return mPeriodicIO.climber_demand;
    }

    public boolean isRunningOpenLoop() {
        return mIsOpenLoop;
    }

    public boolean hasEmergency = false;

    public static class PeriodicIO {

        /* Inputs */
        public double stator_current;
        public double motor_position;
        public double motor_velocity;

        /* Outputs */
        public double climber_demand;
        public boolean deploy_solenoid;
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0.0);
    }
}   
