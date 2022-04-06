package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.ColorSensor;
import com.team1678.frc2022.subsystems.Indexer;
import com.team1678.frc2022.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SystemsTab extends ShuffleboardTabBase {

	private Limelight mLimelight = Limelight.getInstance();
	private Indexer mIndexer = Indexer.getInstance();
	private ColorSensor mColorSensor = ColorSensor.getInstance();

	private NetworkTableEntry mLimelightHasTarget;
	private NetworkTableEntry mLimelightLatency;
	private NetworkTableEntry mLimelightTX;
	private NetworkTableEntry mLimelightTY;
	private NetworkTableEntry mColorSensorProximity;
	private NetworkTableEntry mColorSensorBallColor;
	private NetworkTableEntry mColorSensorHasBall;
	private NetworkTableEntry mForwardBreak;
	private NetworkTableEntry mBottomBreak;
	private NetworkTableEntry mTopBreak;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("System Check");

		mLimelightHasTarget = mTab
				.add("Limelight Has Target", false)
				.withSize(2, 1)
				.withPosition(0, 0)
				.getEntry();
		mLimelightLatency = mTab
				.add("Limelight Latency", 0.0)
				.withSize(2, 1)
				.withPosition(2, 0)
				.getEntry();
		mLimelightTX = mTab
				.add("tx", 0.0)
				.withSize(1, 1)
				.withPosition(4, 0)
				.getEntry();
		mLimelightTY = mTab
				.add("ty", 0.0)
				.withSize(1, 1)
				.withPosition(5, 0)
				.getEntry();
		mColorSensorProximity = mTab
				.add("Color Sensor Proximity", 0.0)
				.withSize(2, 1)
				.withPosition(0, 1)
				.getEntry();
		mColorSensorBallColor = mTab
				.add("Color Sensor Ball Color", "N/A")
				.withSize(2, 1)
				.withPosition(2, 1)
				.getEntry();
		mColorSensorHasBall = mTab
				.add("Color Sensor Has Ball", false)
				.withSize(2, 1)
				.withPosition(4, 1)
				.getEntry();
		mForwardBreak = mTab
				.add("Forward Beam Break", false)
				.withSize(2, 1)
				.withPosition(0, 2)
				.getEntry();
		mBottomBreak = mTab
				.add("Bottom Beam Break", false)
				.withSize(2, 1)
				.withPosition(2, 2)
				.getEntry();
		mTopBreak = mTab
				.add("Top Beam Break", false)
				.withSize(2, 1)
				.withPosition(4, 2)
				.getEntry();
	}

	@Override
	public void update() {
		mLimelightHasTarget.setBoolean(mLimelight.hasTarget());
		mLimelightLatency.setDouble(mLimelight.getLatency());
		mLimelightTX.setDouble(mLimelight.getOffset()[0]);
		mLimelightTY.setDouble(mLimelight.getOffset()[1]);

		mColorSensorProximity.setDouble(mColorSensor.getDistance());
		mColorSensorBallColor.setString(mColorSensor.getMatchedColor());
		mColorSensorHasBall.setBoolean(mColorSensor.hasBall());

		mForwardBreak.setBoolean(mColorSensor.getForwardBeamBreak());
		mBottomBreak.setBoolean(mIndexer.getBottomBeamBreak());
		mTopBreak.setBoolean(mIndexer.getTopBeamBreak());

	}

}
