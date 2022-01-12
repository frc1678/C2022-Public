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

    public TalonFX mMain;

    private Shooter() {
        mMain = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_ID);
        mMain.setInverted(false);
        mMain.setNeutralMode(NeutralMode.Coast);
        /* Tuning Values */
        mMain.config_kP(0, Constants.ShooterConstants.kShooterP, Constants.kLongCANTimeoutMs);
        mMain.config_kI(0, Constants.ShooterConstants.kShooterI, Constants.kLongCANTimeoutMs);
        mMain.config_kD(0, Constants.ShooterConstants.kShooterD, Constants.kLongCANTimeoutMs);
        mMain.config_kF(0, Constants.ShooterConstants.kShooterF, Constants.kLongCANTimeoutMs);
        mMain.config_IntegralZone(0, (int) (200.0 / Constants.ShooterConstants.kFlywheelVelocityConversion));
        mMain.selectProfileSlot(0, 0);
        mMain.configClosedloopRamp(0.1);
        /* Current and voltage limits */
        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mMain.configSupplyCurrentLimit(curr_lim);
        mMain.configVoltageCompSaturation(12, Constants.kLongCANTimeoutMs);
        mMain.enableVoltageCompensation(true);
        /* Use integrated encoder for velocity control */
        mMain.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

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
        mPeriodicIO.flywheel_current = mMain.getSupplyCurrent();
        mPeriodicIO.flywheel_voltage = mMain.getMotorOutputVoltage();
        mPeriodicIO.flywheel_velocity = mMain.getSelectedSensorVelocity()
                * Constants.ShooterConstants.kFlywheelVelocityConversion;
    }

    @Override
    public void writePeriodicOutputs() {
        if (mIsOpenLoop) {
            mMain.set(ControlMode.PercentOutput, mPeriodicIO.flywheel_demand);
        } else {
            mMain.set(ControlMode.Velocity, 0.0);
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

    public synchronized boolean spunUp() {
        if (mPeriodicIO.flywheel_demand > 0) {
            return Util.epsilonEquals(mPeriodicIO.flywheel_demand, mPeriodicIO.flywheel_velocity,
                    Constants.ShooterConstants.kFlywheelTolerance);
        }
        return false;
    }

    public static class PeriodicIO {
        double timestamp;
        /* Inputs */
        public double flywheel_velocity;
        public double flywheel_voltage;
        public double flywheel_current;
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
