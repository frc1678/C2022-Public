package com.team1678.frc2022;

import java.net.IDN;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.team1678.frc2022.Constants.EjectorConstants;
import com.team1678.frc2022.Constants.IndexerConstants;
import com.team1678.frc2022.subsystems.Ejector;
import com.team1678.frc2022.subsystems.Indexer;
import com.team1678.frc2022.subsystems.Intake;
import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.Shooter;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

public class ShuffleBoardInteractions {

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance; 

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    /* Subsystem Dependencies */
    private final Limelight mLimelight;
    private final Swerve mSwerve;
    private final SwerveModule[] mSwerveModules;
    private final Intake mIntake;
    private final Shooter mShooter;
    private final Indexer mIndexer;

    /* Status Variable */
    private double lastCancoderUpdate = 0.0;

    /* Tabs */
    private ShuffleboardTab VISION_TAB;
    private ShuffleboardTab SWERVE_TAB;
    private ShuffleboardTab PID_TAB;
    private ShuffleboardTab INTAKE_TAB;
    private ShuffleboardTab SHOOTER_TAB;
    private ShuffleboardTab INDEXER_TAB;

    /* ENTRIES */

    /* Intake */
    private final NetworkTableEntry mIntakeCurrent;
    private final NetworkTableEntry mIntakeState;
    private final NetworkTableEntry mIntakeVoltage;
    private final NetworkTableEntry mIntakeDemand;
    private final NetworkTableEntry mIntakeDeployed;

    /* Shooter */
    private final NetworkTableEntry mFlywheelRPM;
    private final NetworkTableEntry mAcceleratorRPM;

    // private final NetworkTableEntry mFlywheelManualPIDToggle;
    // private final NetworkTableEntry mFlywheelP;
    // private final NetworkTableEntry mFlywheelI;
    // private final NetworkTableEntry mFlywheelD;
    // private final NetworkTableEntry mFlywheelF;

    /* Vision */
    private final NetworkTableEntry mSeesTarget;
    private final NetworkTableEntry mLimelightOk;
    private final NetworkTableEntry mLimelightLatency;
    private final NetworkTableEntry mLimelightDt;
    private final NetworkTableEntry mLimelightTx;
    private final NetworkTableEntry mLimelightTy;

    /* Swerve Modules */
    private final String[] kSwervePlacements = {"Front Left", "Front Right", "Back Left", "Back Right"};
    private final ShuffleboardLayout[] mSwerveAngles = new ShuffleboardLayout[4];
    private final NetworkTableEntry[] mSwerveCancoders = new NetworkTableEntry[4];
    private final NetworkTableEntry[] mSwerveIntegrated = new NetworkTableEntry[4];
    private final NetworkTableEntry[] mSwerveDrivePercent = new NetworkTableEntry[4];
    private final NetworkTableEntry[] mModuleAngleCurrent = new NetworkTableEntry[4];
    private final NetworkTableEntry[] mModuleAngleGoals = new NetworkTableEntry[4];
    
    private final NetworkTableEntry mSwerveOdometryX;
    private final NetworkTableEntry mSwerveOdometryY;
    private final NetworkTableEntry mSwerveOdometryRot;
    private final NetworkTableEntry mPIDEnableToggle;
    private final NetworkTableEntry mDesiredAngleP;
    private final NetworkTableEntry mDesiredAngleI;
    private final NetworkTableEntry mDesiredAngleD;
    private final NetworkTableEntry mCurrentAngleP;
    private final NetworkTableEntry mCurrentAngleI;
    private final NetworkTableEntry mCurrentAngleD;

    /* INDEXER */
    private final NetworkTableEntry mOuttakeCurrent;


    public ShuffleBoardInteractions() {
        /* Get Subsystems */
        mLimelight = Limelight.getInstance();
        mSwerve = Swerve.getInstance();
        mSwerveModules = Swerve.getInstance().mSwerveMods;
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();
        mIndexer = Indexer.getInstance();

        /* Get Tabs */
        VISION_TAB = Shuffleboard.getTab("Vision");
        SWERVE_TAB = Shuffleboard.getTab("Swerve");
        PID_TAB = Shuffleboard.getTab("Module PID");
        INTAKE_TAB = Shuffleboard.getTab("Intake");
        SHOOTER_TAB = Shuffleboard.getTab("Shooter");
        INDEXER_TAB = Shuffleboard.getTab("Indexer");
        
        /* Create Entries */
        mLimelightOk = VISION_TAB
            .add("Limelight OK", false)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();        
        mSeesTarget = VISION_TAB
            .add("Limelight Sees Target", false)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();
        mLimelightLatency = VISION_TAB
            .add("Limelight Latency", -1.0)
            .withPosition(2, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        mLimelightDt = VISION_TAB
            .add("Limelight Loop Time", -1.0)
            .withPosition(4, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        mLimelightTx = VISION_TAB
            .add("Limelight TX", 0.0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();
        mLimelightTy = VISION_TAB
            .add("Limelight TY", 0.0)
            .withPosition(1, 1)
            .withSize(1, 1)
            .getEntry();

        for (int i = 0; i < mSwerveCancoders.length; i++) {
            mSwerveAngles[i] = SWERVE_TAB
                .getLayout("Module " + i + " Angle", BuiltInLayouts.kGrid)
                .withSize(2, 2)
                .withPosition(i * 2, 0);
            mSwerveCancoders[i] = mSwerveAngles[i].add("Cancoder", 0.0)
                .withPosition(0, 0)
                .withSize(5, 1)
                .getEntry();
            mSwerveAngles[i].add("Location", kSwervePlacements[i])
                .withPosition(1, 0)
                .withSize(5, 1);
            mSwerveIntegrated[i] = mSwerveAngles[i].add("Integrated", 0.0)
                .withPosition(0, 1)
                .withSize(5, 1)
                .getEntry();
            mSwerveAngles[i].add("Offset", mSwerve.mSwerveMods[i].angleOffset)
                .withPosition(0, 2)
                .withSize(5, 1)
                .getEntry();
            mSwerveDrivePercent[i] = SWERVE_TAB
                .add("Swerve Module " + i + " MPS ", 0.0)
                .withPosition(i * 2, 2)
                .withSize(2, 1)
                .getEntry();
            mModuleAngleCurrent[i] = PID_TAB.add("Module " + i + " Current", 0.0)
                .withPosition(i, 2)
                .withSize(1, 1)
                .getEntry();
            mModuleAngleGoals[i] = PID_TAB.add("Module " + i + " Target", 0.0)
                .withPosition(i, 3)
                .withSize(1, 1)
                .getEntry();
        }

        mSwerveOdometryX = SWERVE_TAB
            .add("Odometry X", 0)
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();
        mSwerveOdometryY = SWERVE_TAB
            .add("Odometry Y", 0)
            .withPosition(2, 3)
            .withSize(2, 1)
            .getEntry();
        mSwerveOdometryRot = SWERVE_TAB
            .add("Pigeon Angle", 0)
            .withPosition(4, 3)
            .withSize(2, 1)
            .getEntry();
        mPIDEnableToggle = PID_TAB
            .add("Apply PID", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(3, 0)
            .withSize(2, 1)
            .getEntry();

        TalonFXConfiguration currentAngleValues = CTREConfigs.swerveAngleFXConfig();

        mDesiredAngleP = PID_TAB
            .add("Wanted P", currentAngleValues.slot0.kP)
            .withPosition(0,0)
            .withSize(1, 1)
            .getEntry();
        mDesiredAngleI = PID_TAB
            .add("Wanted I", currentAngleValues.slot0.kI)
            .withPosition(1,0)
            .withSize(1, 1)
            .getEntry();
        mDesiredAngleD = PID_TAB
            .add("Wanted D", currentAngleValues.slot0.kD)
            .withPosition(2,0)
            .withSize(1, 1)
            .getEntry();
        mCurrentAngleP = PID_TAB
            .add("Current P", 0.0)
            .withPosition(0,1)
            .withSize(1, 1)
            .getEntry();
        mCurrentAngleI = PID_TAB
            .add("Current I", 0.0)
            .withPosition(1,1)
            .withSize(1, 1)
            .getEntry();
        mCurrentAngleD = PID_TAB
            .add("Current D", 0.0)
            .withPosition(2,1)
            .withSize(1, 1)
            .getEntry();
        
        /* INTAKE */
        mIntakeCurrent = INTAKE_TAB
            .add("Intake Current", mIntake.mPeriodicIO.intake_current)
            .getEntry();
        mIntakeState = INTAKE_TAB
            .add("Intake State", mIntake.getState().toString())
            .getEntry();
        mIntakeVoltage = INTAKE_TAB
            .add("Intake Voltage", mIntake.mPeriodicIO.intake_voltage)
            .getEntry();
        mIntakeDemand = INTAKE_TAB
                .add("Intake Demand", mIntake.mPeriodicIO.intake_demand)
                .getEntry();
        mIntakeDeployed = INTAKE_TAB
                .add("Intake Deployed", mIntake.mPeriodicIO.deploy)
                .getEntry();
                
        /* Shooter */
        mFlywheelRPM = SHOOTER_TAB
                .add("Shooter RPM", 0.0)
                .withSize(2, 1)
                .getEntry();

        mAcceleratorRPM = SHOOTER_TAB
                .add("Accelerator RPM", 0.0)
                .withSize(2, 1)
                .getEntry();

        /* INDEXER */
        mOuttakeCurrent = INDEXER_TAB
            .add("Outtake Current", 0.0)
            .getEntry();
    }

    public void update() {
        
        /* Intake */
        mIntakeState.setString(mIntake.getState().toString());
        mIntakeVoltage.setDouble(mIntake.getIntakeVoltage());
        mIntakeDemand.setDouble(mIntake.getIntakeDemand());
        mIntakeDeployed.setBoolean(mIntake.getWantDeploy());
        mIntakeCurrent.setDouble(mIntake.getIntakeCurrent());

        /* Shooter */

        mFlywheelRPM.setDouble(truncate(mShooter.getFlywheelRPM()));
        mAcceleratorRPM.setDouble(truncate(mShooter.getAcceleratorRPM()));
        
        /* Vision */
        mSeesTarget.setBoolean(mLimelight.hasTarget());
        mLimelightOk.setBoolean(mLimelight.limelightOK());
        mLimelightLatency.setDouble(mLimelight.getLatency());
        mLimelightDt.setDouble(mLimelight.getDt());
        mLimelightTx.setDouble(mLimelight.getOffset()[0]);
        mLimelightTy.setDouble(mLimelight.getOffset()[1]);

        /* Indexer */
        mOuttakeCurrent.setDouble(mIndexer.getOuttakeDemand());
        
        /* SWERVE */

        // Update cancoders at a slower period to avoid stale can frames
        double dt = Timer.getFPGATimestamp();
        if (dt > lastCancoderUpdate + 0.1) {
            for (int i = 0; i < mSwerveCancoders.length; i++) {
                mSwerveCancoders[i].setDouble(truncate(mSwerveModules[i].getCanCoder().getDegrees()));
            }
            lastCancoderUpdate = dt;
        }
        
        for (int i = 0; i < mSwerveCancoders.length; i++) {
            mSwerveIntegrated[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getState().angle.getDegrees(), 0, 360)));
            mSwerveDrivePercent[i].setDouble(truncate(mSwerveModules[i].getState().speedMetersPerSecond));

            mModuleAngleCurrent[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getState().angle.getDegrees(), 0, 360)));
            mModuleAngleGoals[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getTargetAngle(), 0, 360)));

        }
        mSwerveOdometryX.setDouble(truncate(mSwerve.getPose().getX()));
        mSwerveOdometryY.setDouble(truncate(mSwerve.getPose().getY()));
        mSwerveOdometryRot.setDouble(truncate(MathUtil.inputModulus(mSwerve.getPose().getRotation().getDegrees(), 0, 360)));

        if(mPIDEnableToggle.getValue().getBoolean()) {
            mSwerve.setAnglePIDValues(mDesiredAngleP.getValue().getDouble(), mDesiredAngleI.getValue().getDouble(), mDesiredAngleD.getValue().getDouble());
        }
        double[] currentPIDVals = mSwerve.getAnglePIDValues(0);
        mCurrentAngleP.setDouble(currentPIDVals[0]);
        mCurrentAngleI.setDouble(currentPIDVals[1]);
        mCurrentAngleD.setDouble(currentPIDVals[2]);
    }

    /* Truncates number to 2 decimal places for cleaner numbers */
    private double truncate(double number) {
        return Math.floor(number * 100) / 100;
    }
}
 