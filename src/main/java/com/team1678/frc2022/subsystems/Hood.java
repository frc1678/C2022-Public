package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.team1678.frc2022.Constants;
import com.team254.lib.util.Util;

public class Hood extends ServoMotorSubsystem {

    private static Hood mInstance;

    private boolean mHomed = false;

    public static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    private Hood() {
        super(Constants.HoodConstants.kHoodServoConstants);
    }

    public boolean isHomed() {
        return mHomed;
    }

    public boolean isAtSetpoint() {
        return Util.epsilonEquals(getPosition(), getSetpoint(), 2.0);
    }

    public void zeroHood() {
        mControlState = ControlState.MOTION_MAGIC;
        mMaster.setSelectedSensorPosition(this.mReverseSoftLimitTicks);
        mHomed = true;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        /*
         * If we're not homed, then either calibrate or do nothing. Otherwise let the hood
         * go to setpoint.
         */
        if (!mHomed) {
            if (mControlState == ControlState.OPEN_LOOP) {
                mMaster.set(ControlMode.PercentOutput, Constants.HoodConstants.kCalibratingVoltage);
            } else {
                mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
            }
        } else {
            super.writePeriodicOutputs();
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
        if (!mHomed) {
            mControlState = ControlState.OPEN_LOOP;
            if (mPeriodicIO.master_stator_current > Constants.HoodConstants.kCalibrationCurrentThreshold) {
                zeroHood();
            }

        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

}
