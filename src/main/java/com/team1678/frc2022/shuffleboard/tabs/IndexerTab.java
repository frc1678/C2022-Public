package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Indexer;
import com.team1678.frc2022.subsystems.Superstructure;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class IndexerTab extends ShuffleboardTabBase {

	private Indexer mIndexer = Indexer.getInstance();
	private Superstructure mSuperstructure = Superstructure.getInstance();

	private GenericEntry mEjectorCurrent;
	private GenericEntry mEjectorDemand;
	private GenericEntry mEjectorVoltage;
	private GenericEntry mTunnelCurrent;
	private GenericEntry mTunnelDemand;
	private GenericEntry mTunnelVoltage;
	private GenericEntry mBallCount;
	private GenericEntry mTopBeamBreak;
	private GenericEntry mBottomBeamBreak;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Indexer");
		/* INDEXER */
		mEjectorCurrent = mTab
				.add("Outtake Current", 0.0)
				.getEntry();
		mEjectorDemand = mTab
				.add("Outtake Demand", 0.0)
				.getEntry();
		mEjectorVoltage = mTab
				.add("Outtake Voltage", 0.0)
				.getEntry();
		mTunnelCurrent = mTab
				.add("Indexer Current", 0.0)
				.getEntry();
		mTunnelDemand = mTab
				.add("Indexer Demand", 0.0)
				.getEntry();
		mTunnelVoltage = mTab
				.add("Indexer Voltage", 0.0)
				.getEntry();
		mTopBeamBreak = mTab
				.add("Top Beam Break Triggered", false)
				.getEntry();
		mBottomBeamBreak = mTab
				.add("Bottom Beam Break Triggered", false)
				.getEntry();
		mBallCount = mTab
				.add("Ball Count", 0.0)
				.getEntry();
	}

	@Override
	public void update() {
		mEjectorCurrent.setDouble(mIndexer.getEjectorCurrent());
		mEjectorDemand.setDouble(mIndexer.getEjectorDemand());
		mEjectorVoltage.setDouble(mIndexer.getEjectorVoltage());

		mTunnelCurrent.setDouble(mIndexer.getTunnelCurrent());
		mTunnelDemand.setDouble(mIndexer.getTunnelDemand());
		mTunnelVoltage.setDouble(mIndexer.getTunnelVoltage());

		mBallCount.setDouble(mSuperstructure.getBallCount());

		mTopBeamBreak.setBoolean(mIndexer.getTopBeamBreak());
		mBottomBeamBreak.setBoolean(mIndexer.getBottomBeamBreak());
	}

}
