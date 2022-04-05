package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.Superstructure;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class OperatorTab extends ShuffleboardTabBase {

    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Limelight mLimelight = Limelight.getInstance();

    private NetworkTableEntry mOperatorShooting;
    private NetworkTableEntry mOperatorSpunup;
    private NetworkTableEntry mOperatorFender;
    private NetworkTableEntry mOperatorSpit;
    private NetworkTableEntry mOperatorVisionAimed;
    private NetworkTableEntry mOperatorClimbMode;
    private NetworkTableEntry mOperatorAutoClimb;
    private NetworkTableEntry mOperatorEjectDisable;
    private NetworkTableEntry mOperatorIntakeOverride;

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Operator");

        mOperatorShooting = mTab
                .add("Shooting", false)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        mOperatorSpunup = mTab
                .add("Spun Up", false)
                .withSize(3, 2)
                .withPosition(5, 1)
                .getEntry();
        mOperatorSpit = mTab
                .add("Spitting", false)
                .withSize(3, 1)
                .withPosition(2, 0)
                .getEntry();
        mOperatorFender = mTab
                .add("Fender", false)
                .withSize(3, 1)
                .withPosition(5, 0)
                .getEntry();
        mOperatorVisionAimed = mTab
                .add("Vision Aimed", false)
                .withSize(6, 2)
                .withPosition(2, 3)
                .getEntry();
        mOperatorClimbMode = mTab
                .add("Climb Mode", false)
                .withSize(2, 2)
                .withPosition(8, 0)
                .getEntry();
        mOperatorAutoClimb = mTab
                .add("Auto Climbing", false)
                .withSize(2, 2)
                .withPosition(8, 2)
                .getEntry();
        mOperatorEjectDisable = mTab
                .add("Eject Disable", false)
                .withSize(1, 1)
                .withPosition(1, 1)
                .getEntry();
        mOperatorIntakeOverride = mTab
                .add("Force Intake", false)
                .withSize(1, 1)
                .withPosition(0, 1)
                .getEntry();
    }

    @Override
    public void update() {
        mOperatorShooting.setBoolean(mSuperstructure.getShooting());
        mOperatorSpunup.setBoolean(mSuperstructure.isSpunUp());
        mOperatorSpit.setBoolean(mSuperstructure.getWantsSpit());
        mOperatorFender.setBoolean(mSuperstructure.getWantsFender());
        mOperatorVisionAimed.setBoolean(mLimelight.isAimed());
        mOperatorClimbMode.setBoolean(mSuperstructure.getInClimbMode());
        mOperatorAutoClimb.setBoolean(mSuperstructure.isAutoClimb());

        mOperatorEjectDisable.setBoolean(mSuperstructure.getEjectDisabled());
        mOperatorIntakeOverride.setBoolean(mSuperstructure.getIntakeOverride());
    }

}
