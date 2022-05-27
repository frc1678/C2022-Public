package com.team1678.frc2022.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;

public class Shooter extends Subsystem {

    private static Shooter mInstance;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    
    // logger
    LogStorage<PeriodicIO> mStorage = null;

    private boolean mIsOpenLoop = false;

    private TalonFX mMaster;
    private TalonFX mSlave;
    
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

        setOpenLoop(0.0);

        // reduce can util
        mSlave.changeMotionControlFramePeriod(255);
        mSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // empty
            }

            @Override
            public void onLoop(double timestamp) {
                // send log data
                SendLog();
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

        mPeriodicIO.slave_current = mSlave.getSupplyCurrent();
        mPeriodicIO.slave_velocity = mSlave.getSelectedSensorVelocity();
        mPeriodicIO.slave_voltage = mSlave.getMotorOutputVoltage();
    }

    @Override
    public void writePeriodicOutputs() {
        if (mIsOpenLoop) {
            // set shooter to open loop to avoid hard slowdown
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.flywheel_demand);
        } else {
            mMaster.set(ControlMode.Velocity,
                    mPeriodicIO.flywheel_demand / Constants.ShooterConstants.kFlywheelVelocityConversion);
        }
        
        mSlave.set(ControlMode.Follower, Ports.FLYWHEEL_MASTER_ID);
    }

    public void setOpenLoop(double flywheelDemand) {
        if (mIsOpenLoop != true) {
            mIsOpenLoop = true;
        }
        mPeriodicIO.flywheel_demand = flywheelDemand <= 12.0 ? flywheelDemand : 12.0;
    }

    public void setVelocity(double demand) {
        if (mIsOpenLoop != false) {
            mIsOpenLoop = false;
        }
        mPeriodicIO.flywheel_demand = demand;
    }

    public synchronized double getFlywheelRPM() {
        return mPeriodicIO.flywheel_velocity * Constants.ShooterConstants.kFlywheelVelocityConversion;
    }

    public synchronized double getFlywheelDemand() {
        return mPeriodicIO.flywheel_demand;
    }

    public synchronized boolean getIsOpenLoop() {
        return mIsOpenLoop;
    }

    public synchronized boolean spunUp() {
        if (mPeriodicIO.flywheel_demand > 0) {
            boolean flywheelSpunUp = Util.epsilonEquals(mPeriodicIO.flywheel_demand,
                                      mPeriodicIO.flywheel_velocity * Constants.ShooterConstants.kFlywheelVelocityConversion,
                                      Constants.ShooterConstants.kFlywheelTolerance);
            return flywheelSpunUp;
        }
        return false;
    }

    public static class PeriodicIO {
        /* Inputs */
        private double timestamp;

        private double flywheel_velocity;
        private double flywheel_voltage;
        private double flywheel_current;
        
        private double slave_velocity;
        private double slave_voltage;
        private double slave_current;

        /* Outputs */
        private double flywheel_demand;
    }

    @Override
    public void stop() {
        // set shooter to open loop to avoid hard slowdown
        setOpenLoop(0.0);
    }

    public boolean checkSystem() {
        return true;
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "SHOOTER_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");

        headers.add("flywheel_demand");

        headers.add("flywheel_velocity");
        headers.add("slave_velocity");

        headers.add("flywheel_voltage");
        headers.add("slave_voltage");

        headers.add("flywheel_current");
        headers.add("slave_current");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(mPeriodicIO.timestamp);

        items.add(mPeriodicIO.flywheel_demand);

        items.add(mPeriodicIO.flywheel_velocity);
        items.add(mPeriodicIO.slave_velocity);

        items.add(mPeriodicIO.flywheel_voltage);
        items.add(mPeriodicIO.slave_voltage);

        items.add(mPeriodicIO.flywheel_current);
        items.add(mPeriodicIO.slave_current);

        // send data to logging storage
        mStorage.addData(items);
    }
}
