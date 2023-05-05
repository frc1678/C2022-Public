package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class IntakeTab extends ShuffleboardTabBase {

	private Intake mIntake = Intake.getInstance();

    private GenericEntry mIntakeState;
    private GenericEntry mIntakeRollerCurrent;
    private GenericEntry mIntakeRollerVoltage;
    private GenericEntry mIntakeRollerDemand;
    private GenericEntry mIntakeDeployCurrent;
    private GenericEntry mIntakeDeployVoltage;
    private GenericEntry mIntakeDeployDemand;
    private GenericEntry mIsForceHolding;

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Intake");
		/* INTAKE */
		mIntakeState = mTab
				.add("Intake State", "N/A")
				.getEntry();
		mIntakeRollerCurrent = mTab
				.add("Roller Current", 0.0)
				.getEntry();
		mIntakeRollerVoltage = mTab
				.add("Roller Voltage", 0.0)
				.getEntry();
		mIntakeRollerDemand = mTab
				.add("Roller Demand", 0.0)
				.getEntry();
		mIntakeDeployCurrent = mTab
				.add("Deploy Current", 0.0)
				.getEntry();
		mIntakeDeployVoltage = mTab
				.add("Deploy Voltage", 0.0)
				.getEntry();
		mIntakeDeployDemand = mTab
				.add("Deploy Demand", 0.0)
				.getEntry();
		mIsForceHolding = mTab
				.add("Is Force Holding", 0.0)
				.getEntry();

	}

	@Override
	public void update() {
        mIntakeState.setString(mIntake.getState().toString());

        mIntakeRollerCurrent.setDouble(mIntake.getRollerCurrent());
        mIntakeRollerVoltage.setDouble(mIntake.getRollerVoltage());
        mIntakeRollerDemand.setDouble(mIntake.getRollerDemand());

        mIntakeDeployCurrent.setDouble(mIntake.getDeployCurrent());
        mIntakeDeployVoltage.setDouble(mIntake.getDeployVoltage());
        mIntakeDeployDemand.setDouble(mIntake.getDeployDemand());
        mIsForceHolding.setBoolean(mIntake.getForceHoldIntake());
	}

}
