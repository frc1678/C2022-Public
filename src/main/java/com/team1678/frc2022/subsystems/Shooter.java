package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {

    private static Shooter mInstance;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private boolean mIsOpenLoop = false;

    private TalonFX mMaster;
    private TalonFX mSlave;
    
    private TalonFX mAccelerator;

    private Shooter() {

        /* MAIN FLYWHEEl */
        mMaster = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_MASTER_ID);

        mMaster.setInverted(true);
        mMaster.setNeutralMode(NeutralMode.Coast);

        /* Tuning Values */
        mMaster.config_kP(0, Constants.ShooterConstants.kShooterP, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.ShooterConstants.kShooterI, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.ShooterConstants.kShooterD, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.ShooterConstants.kShooterF, Constants.kLongCANTimeoutMs);
        mMaster.config_IntegralZone(0, (int) (200.0 / Constants.ShooterConstants.kFlywheelVelocityConversion));
        mMaster.selectProfileSlot(0, 0);
        mMaster.configClosedloopRamp(0.1);

        /* Current and voltage limits */
        SupplyCurrentLimitConfiguration main_curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mMaster.configSupplyCurrentLimit(main_curr_lim);
        mMaster.configVoltageCompSaturation(12, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        /* Use integrated encoder for velocity control */
        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        /* FLYWHEEL SLAVE */
        mSlave = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_SLAVE_ID);
        mSlave.setInverted(true);


        /* ACCELERATOR */
        mAccelerator = TalonFXFactory.createDefaultTalon(Ports.ACCELERATOR_ID);
        mAccelerator.setInverted(true);
        mAccelerator.setNeutralMode(NeutralMode.Coast);

        mAccelerator.config_kP(0, Constants.ShooterConstants.kAcceleratorP, Constants.kLongCANTimeoutMs);
        mAccelerator.config_kI(0, Constants.ShooterConstants.kAcceleratorI, Constants.kLongCANTimeoutMs);
        mAccelerator.config_kD(0, Constants.ShooterConstants.kAcceleratorD, Constants.kLongCANTimeoutMs);
        mAccelerator.config_kF(0, Constants.ShooterConstants.kAcceleratorF, Constants.kLongCANTimeoutMs);
        mAccelerator.config_IntegralZone(0, (int) (200.0 / Constants.ShooterConstants.kFlywheelVelocityConversion));
        mAccelerator.selectProfileSlot(0, 0);
        mAccelerator.configClosedloopRamp(0.1);

        /* Current and voltage limits */
        SupplyCurrentLimitConfiguration accel_curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mAccelerator.configSupplyCurrentLimit(accel_curr_lim);
        mAccelerator.configVoltageCompSaturation(12, Constants.kLongCANTimeoutMs);
        mAccelerator.enableVoltageCompensation(true);

        setOpenLoop(0.0, 0.0);

        // reduce can util

        // mMaster.changeMotionControlFramePeriod(255);
        // mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        // mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        mAccelerator.changeMotionControlFramePeriod(255);
        mAccelerator.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mAccelerator.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        mSlave.changeMotionControlFramePeriod(255);
        mSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.flywheel_current = mMaster.getSupplyCurrent();
        mPeriodicIO.flywheel_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.flywheel_velocity = mMaster.getSelectedSensorVelocity();

        mPeriodicIO.accelerator_current = mAccelerator.getSupplyCurrent();
        mPeriodicIO.accelerator_voltage = mAccelerator.getMotorOutputVoltage();
        mPeriodicIO.accelerator_velocity = mAccelerator.getSelectedSensorVelocity();

        mPeriodicIO.slave_current = mSlave.getSupplyCurrent();
        mPeriodicIO.slave_velocity = mSlave.getSelectedSensorVelocity();
        mPeriodicIO.slave_voltage = mSlave.getMotorOutputVoltage();
    }

    @Override
    public void writePeriodicOutputs() {
        if (mIsOpenLoop) {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.flywheel_demand);
            mAccelerator.set(ControlMode.PercentOutput, mPeriodicIO.accelerator_demand);
        } else {
            SmartDashboard.putNumber("Flywheel Input Demand",
                    mPeriodicIO.flywheel_demand / Constants.ShooterConstants.kFlywheelVelocityConversion);
            SmartDashboard.putNumber("Accelerator Input Demand",
                    mPeriodicIO.accelerator_demand / Constants.ShooterConstants.kFlywheelVelocityConversion);
            mMaster.set(ControlMode.Velocity,
                    mPeriodicIO.flywheel_demand / Constants.ShooterConstants.kFlywheelVelocityConversion);
            mAccelerator.set(ControlMode.Velocity,
                    mPeriodicIO.accelerator_demand / Constants.ShooterConstants.kAccleratorVelocityConversion);
        }
        
        mSlave.set(ControlMode.Follower, Ports.FLYWHEEL_MASTER_ID);
    }

    public void setOpenLoop(double flywheelDemand, double acceleratorDemand) {
        if (mIsOpenLoop != true) {
            mIsOpenLoop = true;
        }
        mPeriodicIO.flywheel_demand = flywheelDemand <= 12.0 ? flywheelDemand : 12.0;
        mPeriodicIO.accelerator_demand = acceleratorDemand <= 12.0 ? acceleratorDemand : 12.0;
    }

    public void setVelocity(double demand, double accleratorDemand) {
        if (mIsOpenLoop != false) {
            mIsOpenLoop = false;
        }
        mPeriodicIO.flywheel_demand = demand;
        mPeriodicIO.accelerator_demand = accleratorDemand;
    }

    public synchronized double getFlywheelRPM() {
        return mPeriodicIO.flywheel_velocity * Constants.ShooterConstants.kFlywheelVelocityConversion;
    }

    public synchronized double getAcceleratorRPM() {
        return mPeriodicIO.accelerator_velocity * Constants.ShooterConstants.kAccleratorVelocityConversion;
    }
    
    public synchronized double getFlywheelDemand() {
        return mPeriodicIO.flywheel_demand;
    }

    public synchronized double getAcceleratorDemand() {
        return mPeriodicIO.accelerator_demand;
    }

    public synchronized boolean getIsOpenLoop() {
        return mIsOpenLoop;
    }

    public synchronized boolean spunUp() {
        if (mPeriodicIO.flywheel_demand > 0) {
            boolean flywheelSpunUp = Util.epsilonEquals(mPeriodicIO.flywheel_demand,
                                      mPeriodicIO.flywheel_velocity * Constants.ShooterConstants.kFlywheelVelocityConversion,
                                      Constants.ShooterConstants.kFlywheelTolerance);
            boolean acceleratorSpunUp = Util.epsilonEquals(mPeriodicIO.accelerator_demand,
                                      mPeriodicIO.accelerator_velocity * Constants.ShooterConstants.kAccleratorVelocityConversion,
                                      Constants.ShooterConstants.kFlywheelTolerance);
            return flywheelSpunUp/* && acceleratorSpunUp*/;
        }
        return false;
    }

    public static class PeriodicIO {
        /* Inputs */
        public double timestamp;

        public double flywheel_velocity;
        public double flywheel_voltage;
        public double flywheel_current;
        
        public double slave_velocity;
        public double slave_voltage;
        public double slave_current;

        public double accelerator_velocity;
        public double accelerator_voltage;
        public double accelerator_current;

        /* Outputs */
        public double flywheel_demand;
        public double accelerator_demand;
    }

    @Override
    public void stop() {
        /* Set motor to open loop to avoid hard slowdown */
        setOpenLoop(0.0, 0.0);
    }

    public boolean checkSystem() {
        return true;
    }
}
