package com.team1678.frc2022.shuffleboard.tabs;

import com.team1678.frc2022.drivers.SwerveModule;
import com.team1678.frc2022.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveTab extends ShuffleboardTabBase {

    private Swerve mSwerve = Swerve.getInstance();

    private final SwerveModule[] mSwerveModules;

    private String[] kSwervePlacements = {"Front Left", "Front Right", "Back Left", "Back Right"};
    private ShuffleboardLayout[] mSwerveLayouts = new ShuffleboardLayout[4];
    private NetworkTableEntry[] mSwerveCancoders = new NetworkTableEntry[4];
    private NetworkTableEntry[] mSwerveIntegrated = new NetworkTableEntry[4];
    private NetworkTableEntry[] mSwerveDrivePercent = new NetworkTableEntry[4];

    private NetworkTableEntry mSwerveOdometryX;
    private NetworkTableEntry mSwerveOdometryY;
    private NetworkTableEntry mSwerveOdometryRot;

    public SwerveTab() {
        super();
        mSwerveModules = mSwerve.mSwerveMods;
    }

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Swerve");

        for (int i = 0; i < mSwerveCancoders.length; i++) {
            mSwerveLayouts[i] = mTab
                .getLayout("Module " + i + " Angle", BuiltInLayouts.kGrid)
                .withSize(2, 2)
                .withPosition(i * 2, 0);
            mSwerveCancoders[i] = mSwerveLayouts[i].add("Cancoder", 0.0)
                .withPosition(0, 0)
                .withSize(5, 1)
                .getEntry();
            mSwerveLayouts[i].add("Location", kSwervePlacements[i])
                .withPosition(1, 0)
                .withSize(5, 1);
            mSwerveIntegrated[i] = mSwerveLayouts[i].add("Integrated", 0.0)
                .withPosition(0, 1)
                .withSize(5, 1)
                .getEntry();
            mSwerveLayouts[i].add("Offset", mSwerve.mSwerveMods[i].angleOffset)
                .withPosition(0, 2)
                .withSize(5, 1)
                .getEntry();
            mSwerveDrivePercent[i] = mTab
                .add("Swerve Module " + i + " MPS ", 0.0)
                .withPosition(i * 2, 2)
                .withSize(2, 1)
                .getEntry();
        }

        mSwerveOdometryX = mTab
            .add("Odometry X", 0)
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();
        mSwerveOdometryY = mTab
            .add("Odometry Y", 0)
            .withPosition(2, 3)
            .withSize(2, 1)
            .getEntry();
        mSwerveOdometryRot = mTab
            .add("Pigeon Angle", 0)
            .withPosition(4, 3)
            .withSize(2, 1)
            .getEntry();
    }

    @Override
    public void update() {
        for (int i = 0; i < mSwerveCancoders.length; i++) {
            // mSwerveCancoders[i].setDouble(truncate(mSwerveModules[i].getCanCoder().getDegrees()));
            mSwerveIntegrated[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getState().angle.getDegrees(), 0, 360)));
            mSwerveDrivePercent[i].setDouble(truncate(mSwerveModules[i].getState().speedMetersPerSecond));
        }
        
        mSwerveOdometryX.setDouble(truncate(mSwerve.getPose().getX()));
        mSwerveOdometryY.setDouble(truncate(mSwerve.getPose().getY()));
        mSwerveOdometryRot.setDouble(truncate(MathUtil.inputModulus(mSwerve.getPose().getRotation().getDegrees(), 0, 360)));

    }

}
