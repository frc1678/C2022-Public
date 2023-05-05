package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Superstructure;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ManualShooterTab extends ShuffleboardTabBase {

	private Superstructure mSuperstructure = Superstructure.getInstance();

	private GenericEntry mManualShooterRPM;
	private GenericEntry mManualHoodAngle;
	private GenericEntry mShootingSetpointsEnableToggle;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Manual Params");
		/* CANdle */
        mManualShooterRPM = mTab
            .add("Manual Shooter Goal", 0.0)
            .withSize(2, 1)
            .getEntry();
        mManualHoodAngle = mTab
            .add("Manual Hood Goal", 0.0)
            .withSize(2, 1)
            .getEntry();
        mShootingSetpointsEnableToggle = mTab
            .add("Apply Manual Shooting Params", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(2, 1)
            .getEntry();

	}

	@Override
	public void update() {
        if(mShootingSetpointsEnableToggle.get().getBoolean()) {
            mSuperstructure.setShootingParameters(mManualShooterRPM.getDouble(0.0), mManualHoodAngle.getDouble(0.0));
        }
	}

}
