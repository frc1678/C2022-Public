package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.team1678.frc2022.Constants;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends ServoMotorSubsystem {

    private static Hood mInstance;

    private boolean mHomed = true;

    public static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    private Hood() {
        super(Constants.HoodConstants.kHoodServoConstants);
        mMaster.setInverted(false);
        mMaster.setSelectedSensorPosition(
                getHoodDegreesToTicks(Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit));

    }

    public boolean isHomed() {
        return mHomed;
    }

    public boolean isAtSetpoint() {
        return Util.epsilonEquals(getPosition(), getSetpoint(), 2.0);
    }

    public void zeroHood() {
        mControlState = ControlState.MOTION_MAGIC;
        mMaster.setSelectedSensorPosition(getHoodDegreesToTicks(Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit));
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

        outputTelemetry(); // TODO Remove this
        mControlState = ControlState.MOTION_MAGIC;
        super.readPeriodicInputs();
        /*
        if (!mHomed) {
            mControlState = ControlState.OPEN_LOOP;
            if (mPeriodicIO.master_stator_current > Constants.HoodConstants.kCalibrationCurrentThreshold) {
                zeroHood();
            }
        }
        */
    }

    public void outputTelemetry() {
        super.outputTelemetry();

        SmartDashboard.putBoolean(mConstants.kName + " Calibrated", !mHomed);
        SmartDashboard.putString("Hood Control State", mControlState.toString());
        SmartDashboard.putBoolean("Hood at Homing Location", atHomingLocation());
        SmartDashboard.putNumber("Hood Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Hood Current", mPeriodicIO.master_stator_current);
        SmartDashboard.putNumber("Hood Angle", getHoodAngle());
    }

    public synchronized double getHoodAngle() {
        return getTicksToHoodDegrees(mMaster.getSelectedSensorPosition());
    }

    public double getTicksToHoodDegrees(double ticks) {
        return ticks / Constants.HoodConstants.kHoodServoConstants.kTicksPerUnitDistance;
    }

    public double getHoodDegreesToTicks(double degrees) {
        return degrees * Constants.HoodConstants.kHoodServoConstants.kTicksPerUnitDistance;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

}
