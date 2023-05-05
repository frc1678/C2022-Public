package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.LEDs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class LedTab extends ShuffleboardTabBase {

	private LEDs mLEDs = LEDs.getInstance();

	private GenericEntry mTopLEDState;
	private GenericEntry mBottomLEDState;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Leds");
		/* CANdle */
		mTopLEDState = mTab
				.add("Top LEDs State", "N/A")
				.withSize(2, 1)
				.getEntry();

		mBottomLEDState = mTab
				.add("Bottom LEDs State", "N/A")
				.withSize(2, 1)
				.getEntry();

	}

	@Override
	public void update() {
        mTopLEDState.setString(mLEDs.getTopState().getName());
        mBottomLEDState.setString(mLEDs.getBottomState().getName());
	}

}
