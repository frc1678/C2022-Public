package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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

    private Shooter() {
        mMaster = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_MASTER_ID);
        mMaster.setInverted(false);
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
        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mMaster.configSupplyCurrentLimit(curr_lim);
        mMaster.configVoltageCompSaturation(12, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
        /* Use integrated encoder for velocity control */
        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mSlave = TalonFXFactory.createPermanentSlaveTalon(Ports.FLYWHEEL_SLAVE_ID, Ports.FLYWHEEL_MASTER_ID);
        mSlave.setInverted(true);

        setOpenLoop(0.0);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                setVelocity(500);
                //setOpenLoop(2.0);
                SmartDashboard.putBoolean("Open Loop Shooter", mIsOpenLoop);
                SmartDashboard.putNumber("Shooter Demand", mPeriodicIO.flywheel_demand);
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
        mPeriodicIO.falcon_current = mMaster.getSupplyCurrent();
        mPeriodicIO.falcon_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.falcon_velocity = mMaster.getSelectedSensorVelocity();
    }

    @Override
    public void writePeriodicOutputs() {
        if (mIsOpenLoop) {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.flywheel_demand);
        } else {
            mMaster.set(ControlMode.Velocity,
                    mPeriodicIO.flywheel_demand / Constants.ShooterConstants.kFlywheelVelocityConversion);
        }
    }

    public void setOpenLoop(double demand) {
        if (mIsOpenLoop != true) {
            mIsOpenLoop = true;
        }
        mPeriodicIO.flywheel_demand = demand <= 12.0 ? demand : 12.0;
    }

    public void setVelocity(double demand) {
        if (mIsOpenLoop != false) {
            mIsOpenLoop = false;
        }
        mPeriodicIO.flywheel_demand = demand;
    }

    public synchronized double getShooterRPM() {
        return mPeriodicIO.falcon_velocity * Constants.ShooterConstants.kFlywheelVelocityConversion;
    }

    public synchronized double getKickerRPM() {
        return mPeriodicIO.falcon_velocity * Constants.ShooterConstants.kKickerVelocityConversion;
    }

    public synchronized boolean spunUp() {
        if (mPeriodicIO.flywheel_demand > 0) {
            return Util.epsilonEquals(mPeriodicIO.flywheel_demand, mPeriodicIO.falcon_velocity,
                    Constants.ShooterConstants.kFlywheelTolerance);
        }
        return false;
    }

    public static class PeriodicIO {
        /* Inputs */
        public double timestamp;
        public double falcon_velocity;
        public double falcon_voltage;
        public double falcon_current;
        /* Outputs */
        public double flywheel_demand;
    }

    @Override
    public void stop() {
        /* Set motor to open loop to avoid hard slowdown */
        setOpenLoop(0.0);
    }

    public boolean checkSystem() {
        return true;
    }
}
