package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Shooter;
import com.team1678.frc2022.subsystems.Trigger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShooterTab extends ShuffleboardTabBase {

	private Shooter mShooter = Shooter.getInstance();
	private Trigger mTrigger = Trigger.getInstance();

    private NetworkTableEntry mFlywheelRPM;
    private NetworkTableEntry mShooterOpenLoop;
    private NetworkTableEntry mFlywheelDemand;
    private NetworkTableEntry mTriggerVelocity;
    private NetworkTableEntry mTriggerCurrent;
    private NetworkTableEntry mTriggerDemand;
    private NetworkTableEntry mTriggerVoltage;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Shooter");

        mFlywheelRPM = mTab
                .add("Shooter RPM", 0.0)
                .withSize(2, 1)
                .getEntry();
        mFlywheelDemand = mTab
                .add("Shooter Demand", 0.0)
                .withSize(2, 1)
                .getEntry();
        mTriggerCurrent = mTab
                .add("Trigger Current", 0.0)
                .withSize(2, 1)
                .getEntry();
        mTriggerDemand = mTab
                .add("Trigger Demand", 0.0)
                .withSize(2, 1)
                .getEntry();
        mTriggerVoltage = mTab
                .add("Trigger Voltage", 0.0)
                .withSize(2, 1)
                .getEntry();
        mTriggerVelocity = mTab
                .add("Trigger Velocity", 0.0)
                .withSize(2, 1)
                .getEntry();
        mShooterOpenLoop = mTab
                .add("Shooter Open Loop", false)
                .withSize(2, 1)
                .getEntry();
        
	}

	@Override
	public void update() {
        mFlywheelRPM.setDouble(truncate(mShooter.getFlywheelRPM()));
        mShooterOpenLoop.setBoolean(mShooter.getIsOpenLoop());
        mFlywheelDemand.setDouble(truncate(mShooter.getFlywheelDemand()));

        mTriggerCurrent.setDouble(mTrigger.getTriggerCurrent());
        mTriggerDemand.setDouble(mTrigger.getTriggerDemand());
        mTriggerVoltage.setDouble(mTrigger.getTriggerVoltage());
        mTriggerVelocity.setDouble(mTrigger.getTriggerVelocity());
	}

}
