package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Climber;
import com.team1678.frc2022.subsystems.Superstructure;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ClimberTab extends ShuffleboardTabBase {

	private Climber mClimber = Climber.getInstance();
	private Superstructure mSuperstructure = Superstructure.getInstance();

	private GenericEntry mClimberVelocityRight;
	private GenericEntry mClimberVelocityLeft;
	private GenericEntry mClimberDemandRight;
	private GenericEntry mClimberDemandLeft;
	private GenericEntry mClimberPositionRight;
	private GenericEntry mClimberPositionLeft;
	private GenericEntry mClimberCurrentRight;
	private GenericEntry mClimberCurrentLeft;
	private GenericEntry mClimberHomed;
	private GenericEntry mClimberLeftControlState;
	private GenericEntry mClimberRightControlState;
	private GenericEntry mInClimbMode;
	private GenericEntry mOpenLoopClimbControl;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Climber");
		mInClimbMode = mTab
				.add("Climb Mode", false)
				.getEntry();
		mOpenLoopClimbControl = mTab
				.add("Open Loop Climb Control", false)
				.getEntry();
		mClimberLeftControlState = mTab
				.add("Climber Left Control State", mClimber.mLeftControlState.toString())
				.getEntry();
		mClimberRightControlState = mTab
				.add("Climber Right Control State", mClimber.mRightControlState.toString())
				.getEntry();
		mClimberVelocityRight = mTab
				.add("Right Climber Velocity", 0.0)
				.getEntry();
		mClimberVelocityLeft = mTab
				.add("Left Climber Velocity", 0.0)
				.getEntry();
		mClimberDemandRight = mTab
				.add("Right Climber Demand", 0.0)
				.getEntry();
		mClimberDemandLeft = mTab
				.add("Left Climber Demand", 0.0)
				.getEntry();
		mClimberPositionRight = mTab
				.add("Right Climber Position", 0.0)
				.getEntry();
		mClimberPositionLeft = mTab
				.add("Left Climber Position", 0.0)
				.getEntry();
		mClimberCurrentRight = mTab
				.add("Right Climber Current", 0.0)
				.getEntry();
		mClimberCurrentLeft = mTab
				.add("Left Climber Current", 0.0)
				.getEntry();
		mClimberHomed = mTab
				.add("Climber is Homed", false)
				.getEntry();

	}

	@Override
	public void update() {
        mInClimbMode.setBoolean(mSuperstructure.getInClimbMode());
        mOpenLoopClimbControl.setBoolean(mSuperstructure.isOpenLoopClimbControl());
        mClimberLeftControlState.setString(mClimber.getLeftControlState().toString());
        mClimberRightControlState.setString(mClimber.getRightControlState().toString());

        mClimberVelocityRight.setDouble(mClimber.getClimberVelocityRight());
        mClimberVelocityLeft.setDouble(mClimber.getClimberVelocityLeft());
        
        mClimberDemandRight.setDouble(mClimber.getClimberDemandRight());
        mClimberDemandLeft.setDouble(mClimber.getClimberDemandLeft());

        mClimberPositionRight.setDouble(mClimber.getClimberPositionRight());
        mClimberPositionLeft.setDouble(mClimber.getClimberPositionLeft());

        mClimberCurrentRight.setDouble(mClimber.getClimberCurrentRight());
        mClimberCurrentLeft.setDouble(mClimber.getClimberCurrentLeft());
        
        mClimberHomed.setBoolean(mClimber.getHomed());
	}

}
