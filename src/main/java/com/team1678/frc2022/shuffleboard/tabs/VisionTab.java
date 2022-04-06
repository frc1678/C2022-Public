package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class VisionTab extends ShuffleboardTabBase {

	private Limelight mLimelight = Limelight.getInstance();

	private NetworkTableEntry mSeesTarget;
	private NetworkTableEntry mLimelightOk;
	private NetworkTableEntry mLimelightLatency;
	private NetworkTableEntry mLimelightDt;
	private NetworkTableEntry mLimelightTx;
	private NetworkTableEntry mLimelightTy;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Vision");

		mLimelightOk = mTab
				.add("Limelight OK", false)
				.withPosition(0, 0)
				.withSize(1, 1)
				.getEntry();
		mSeesTarget = mTab
				.add("Limelight Sees Target", false)
				.withPosition(1, 0)
				.withSize(1, 1)
				.getEntry();
		mLimelightLatency = mTab
				.add("Limelight Latency", -1.0)
				.withPosition(2, 0)
				.withSize(2, 2)
				.withWidget(BuiltInWidgets.kTextView)
				.getEntry();
		mLimelightDt = mTab
				.add("Limelight Loop Time", -1.0)
				.withPosition(4, 0)
				.withSize(2, 2)
				.withWidget(BuiltInWidgets.kTextView)
				.getEntry();
		mLimelightTx = mTab
				.add("Limelight TX", 0.0)
				.withPosition(0, 1)
				.withSize(1, 1)
				.getEntry();
		mLimelightTy = mTab
				.add("Limelight TY", 0.0)
				.withPosition(1, 1)
				.withSize(1, 1)
				.getEntry();
	}

	@Override
	public void update() {
		mSeesTarget.setBoolean(mLimelight.hasTarget());
		mLimelightOk.setBoolean(mLimelight.limelightOK());
		mLimelightLatency.setDouble(mLimelight.getLatency());
		mLimelightDt.setDouble(mLimelight.getDt());
		mLimelightTx.setDouble(mLimelight.getOffset()[0]);
		mLimelightTy.setDouble(mLimelight.getOffset()[1]);
	}

}
