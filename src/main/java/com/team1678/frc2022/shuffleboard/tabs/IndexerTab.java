package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Indexer;
import com.team1678.frc2022.subsystems.Superstructure;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class IndexerTab extends ShuffleboardTabBase {

	private Indexer mIndexer = Indexer.getInstance();
	private Superstructure mSuperstructure = Superstructure.getInstance();

	private NetworkTableEntry mEjectorCurrent;
	private NetworkTableEntry mEjectorDemand;
	private NetworkTableEntry mEjectorVoltage;

	private NetworkTableEntry mTunnelCurrent;
	private NetworkTableEntry mTunnelDemand;
	private NetworkTableEntry mTunnelVoltage;

	private NetworkTableEntry mIndexerState;

	private NetworkTableEntry mBallCount;

	private NetworkTableEntry mTopBeamBreak;
	private NetworkTableEntry mBottomBeamBreak;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Indexer");
		/* INDEXER */
		mIndexerState = mTab
				.add("Indexer State", "N/A")
				.getEntry();
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

		mIndexerState.setString(mIndexer.getState().toString());
		mBallCount.setDouble(mSuperstructure.getBallCount());

		mTopBeamBreak.setBoolean(mIndexer.getTopBeamBreak());
		mBottomBeamBreak.setBoolean(mIndexer.getBottomBeamBreak());
	}

}
