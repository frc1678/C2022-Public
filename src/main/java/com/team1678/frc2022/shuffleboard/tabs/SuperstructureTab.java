package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Superstructure;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SuperstructureTab extends ShuffleboardTabBase {

	private Superstructure mSuperstructure = Superstructure.getInstance();

	private NetworkTableEntry mIntaking;
	private NetworkTableEntry mReversing;
	private NetworkTableEntry mRejecting;
	private NetworkTableEntry mEjecting;
	private NetworkTableEntry mPrepping;
	private NetworkTableEntry mShooting;
	private NetworkTableEntry mFenderShot;
	private NetworkTableEntry mSpitShot;

	// goals
	private NetworkTableEntry mIntakeGoal;
	private NetworkTableEntry mShooterGoal;
	private NetworkTableEntry mHoodGoal;

	// additional status vars
	private NetworkTableEntry mSpunUp;
	private NetworkTableEntry mHasTarget;
	private NetworkTableEntry mIsAimed;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Superstructure");
		// actions
		mIntaking = mTab
				.add("Intaking", false)
				.withSize(1, 1)
				.getEntry();
		mReversing = mTab
				.add("Reversing", false)
				.withSize(1, 1)
				.getEntry();
		mRejecting = mTab
				.add("Rejecting", false)
				.withSize(1, 1)
				.getEntry();
		mEjecting = mTab
				.add("Ejecting", false)
				.withSize(1, 1)
				.getEntry();
		mPrepping = mTab
				.add("Prepping", false)
				.withSize(1, 1)
				.getEntry();
		mShooting = mTab
				.add("Shooting", false)
				.withSize(1, 1)
				.getEntry();
		mFenderShot = mTab
				.add("Fender Shot", false)
				.withSize(1, 1)
				.getEntry();
		mSpitShot = mTab
				.add("Spit Shot", false)
				.withSize(1, 1)
				.getEntry();

		// goals
		mIntakeGoal = mTab
				.add("Intake Goal", "N/A")
				.withSize(2, 1)
				.getEntry();
		mShooterGoal = mTab
				.add("Shooter Goal", 0.0)
				.withSize(2, 1)
				.getEntry();
		mHoodGoal = mTab
				.add("Hood Goal", 0.0)
				.withSize(2, 1)
				.getEntry();

		// additional status vars
		mSpunUp = mTab
				.add("Is Spun Up", false)
				.withSize(2, 1)
				.getEntry();
		mHasTarget = mTab
				.add("Has Vision Target", false)
				.withSize(2, 1)
				.getEntry();
		mIsAimed = mTab
				.add("Is Vision Aimed", false)
				.withSize(2, 1)
				.getEntry();

	}

	@Override
	public void update() {
        mIntaking.setBoolean(mSuperstructure.getIntaking());
        mReversing.setBoolean(mSuperstructure.getReversing());
        mRejecting.setBoolean(mSuperstructure.getRejecting());
        mEjecting.setBoolean(mSuperstructure.getEjecting());
        mPrepping.setBoolean(mSuperstructure.getPrepping());
        mShooting.setBoolean(mSuperstructure.getShooting());
        mFenderShot.setBoolean(mSuperstructure.getWantsFender());
        mSpitShot.setBoolean(mSuperstructure.getWantsSpit());

        // update superstructure goal statuses
        mIntakeGoal.setString(mSuperstructure.getIntakeGoal());
        mShooterGoal.setDouble(mSuperstructure.getShooterGoal());
        mHoodGoal.setDouble(mSuperstructure.getHoodGoal());

        // update other status vars
        mSpunUp.setBoolean(mSuperstructure.isSpunUp());
        mHasTarget.setBoolean(mSuperstructure.hasTarget());
        mIsAimed.setBoolean(mSuperstructure.isAimed());
	}

}
