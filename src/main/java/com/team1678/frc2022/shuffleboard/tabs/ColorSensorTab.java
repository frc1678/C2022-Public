package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.ColorSensor;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ColorSensorTab extends ShuffleboardTabBase {

	private ColorSensor mColorSensor = ColorSensor.getInstance();

	private GenericEntry mSensor0;
	private GenericEntry mRValue;
	private GenericEntry mGValue;
	private GenericEntry mBValue;
	private GenericEntry mAllianceColor;
	private GenericEntry mMatchedColor;
	private GenericEntry mForwardBreak;
	private GenericEntry mProximity;
	private GenericEntry mSeesBall;
	private GenericEntry mEject;
	private GenericEntry mTimestamp;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Color Sensor");
		mSensor0 = mTab
				.add("Is Sensor 0 Connected", false)
				.getEntry();
		mRValue = mTab
				.add("Detected R Value", 0.0)
				.getEntry();
		mGValue = mTab
				.add("Detected G Value", 0.0)
				.getEntry();
		mBValue = mTab
				.add("Detected B Value", 0.0)
				.getEntry();

		mAllianceColor = mTab
				.add("Alliance Color", "N/A")
				.getEntry();
		mMatchedColor = mTab
				.add("Matched Color", "N/A")
				.getEntry();
		mForwardBreak = mTab
				.add("Beam Break", false)
				.getEntry();

		mProximity = mTab
				.add("Proximity", 0.0)
				.getEntry();

		mSeesBall = mTab
				.add("Sees Ball", false)
				.getEntry();
		mEject = mTab
				.add("Eject", false)
				.getEntry();

		mTimestamp = mTab
				.add("Timestamp", 0.0)
				.getEntry();

	}

	@Override
	public void update() {
        mSensor0.setBoolean(mColorSensor.getSensor0());
        mRValue.setDouble(mColorSensor.getDetectedRValue());
        mGValue.setDouble(mColorSensor.getDetectedGValue());
        mBValue.setDouble(mColorSensor.getDetectedBValue());
        mBValue.setDouble(mColorSensor.getDetectedBValue());
        
        mAllianceColor.setString(mColorSensor.getAllianceColor().toString());
        mMatchedColor.setString(mColorSensor.getMatchedColor().toString());
        mForwardBreak.setBoolean(mColorSensor.getForwardBeamBreak());

        mProximity.setDouble(mColorSensor.getDistance());

        mSeesBall.setBoolean(mColorSensor.seesBall());
        mEject.setBoolean(mColorSensor.hasOppositeColor());

        mTimestamp.setDouble(mColorSensor.getTimestamp());

	}

}
