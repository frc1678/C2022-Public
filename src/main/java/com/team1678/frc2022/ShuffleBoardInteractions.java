package com.team1678.frc2022;

import com.team1678.frc2022.subsystems.Indexer;
import com.team1678.frc2022.subsystems.Intake;
import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.Shooter;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

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
    private final Superstructure mSuperstructure;

    /* Status Variables */
    private double lastCancoderUpdate = 0.0;

    /* Tabs */
    private ShuffleboardTab VISION_TAB;
    private ShuffleboardTab SWERVE_TAB;
    private ShuffleboardTab PID_TAB;
    private ShuffleboardTab INTAKE_TAB;
    private ShuffleboardTab SHOOTER_TAB;
    private ShuffleboardTab INDEXER_TAB;
    private ShuffleboardTab SUPERSTRUCTURE_TAB;
    private ShuffleboardTab MANUAL_PARAMS;

    /*** ENTRIES ***/
    
    /* SWERVE MODULES */
    private final String[] kSwervePlacements = {"Front Left", "Front Right", "Back Left", "Back Right"};
    private final NetworkTableEntry mSwerveBrakeMode;
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

    /* INTAKE */
    private final NetworkTableEntry mIntakeCurrent;
    private final NetworkTableEntry mIntakeState;
    private final NetworkTableEntry mIntakeVoltage;
    private final NetworkTableEntry mIntakeDemand;
    private final NetworkTableEntry mIntakeDeployed;

    /* INDEXER */
    private final NetworkTableEntry mIndexerState;
    private final NetworkTableEntry mTopBeamBreak;
    private final NetworkTableEntry mBottomBeamBreak;
    private final NetworkTableEntry mBallCount;

    private final NetworkTableEntry mTunnelDemand;
    private final NetworkTableEntry mTunnelVoltage;
    private final NetworkTableEntry mTunnelCurrent;
    private final NetworkTableEntry mTunnelVelocity;

    private final NetworkTableEntry mTriggerDemand;
    private final NetworkTableEntry mTriggerVoltage;
    private final NetworkTableEntry mTriggerCurrent;
    private final NetworkTableEntry mTriggerVelocity;

    /* SHOOTER */
    private final NetworkTableEntry mFlywheelRPM;
    private final NetworkTableEntry mAcceleratorRPM;
    private final NetworkTableEntry mShooterOpenLoop;
    private final NetworkTableEntry mFlywheelDemand;
    private final NetworkTableEntry mAcceleratorDemand;

    /* VISION */
    private final NetworkTableEntry mSeesTarget;
    private final NetworkTableEntry mLimelightOk;
    private final NetworkTableEntry mLimelightLatency;
    private final NetworkTableEntry mLimelightDt;
    private final NetworkTableEntry mLimelightTx;
    private final NetworkTableEntry mLimelightTy;

    /* SUPERSTRUCTURE */
    // actions
    private final NetworkTableEntry mIntaking;
    private final NetworkTableEntry mOuttaking;
    private final NetworkTableEntry mPrepping;
    private final NetworkTableEntry mShooting;
    private final NetworkTableEntry mFender;

    // goals
    private final NetworkTableEntry mIntakeGoal;
    private final NetworkTableEntry mIndexerGoal;
    private final NetworkTableEntry mShooterGoal;
    private final NetworkTableEntry mHoodGoal;

    // additional status vars
    private final NetworkTableEntry mSpunUp;
    private final NetworkTableEntry mHasTarget;
    private final NetworkTableEntry mIsAimed;

    /* MANUAL PARAMS */
    private final NetworkTableEntry mManualShooterRPM;
    private final NetworkTableEntry mManualHoodAngle;
    private final NetworkTableEntry mShootingSetpointsEnableToggle;

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        /* Get Subsystems */
        mSwerve = Swerve.getInstance();
        mSwerveModules = Swerve.getInstance().mSwerveMods;
        mIntake = Intake.getInstance();
        mIndexer = Indexer.getInstance();
        mShooter = Shooter.getInstance();
        mLimelight = Limelight.getInstance();
        mSuperstructure = Superstructure.getInstance();

        /* Get Tabs */
        SWERVE_TAB = Shuffleboard.getTab("Swerve");
        PID_TAB = Shuffleboard.getTab("Module PID");
        INTAKE_TAB = Shuffleboard.getTab("Intake");
        INDEXER_TAB = Shuffleboard.getTab("Indexer");
        SHOOTER_TAB = Shuffleboard.getTab("Shooter");
        VISION_TAB = Shuffleboard.getTab("Vision");
        SUPERSTRUCTURE_TAB = Shuffleboard.getTab("Superstructure");
        MANUAL_PARAMS = Shuffleboard.getTab("Manual Params");
        
        /*** Create Entries ***/

        mSwerveBrakeMode = SWERVE_TAB.add("Swerve Break Mode", false).getEntry();

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
            .add("Intake Current", mIntake.getIntakeCurrent())
            .getEntry();
        mIntakeState = INTAKE_TAB
            .add("Intake State", mIntake.getState().toString())
            .getEntry();
        mIntakeVoltage = INTAKE_TAB
            .add("Intake Voltage", mIntake.getIntakeVoltage())
            .getEntry();
        mIntakeDemand = INTAKE_TAB
                .add("Intake Demand", mIntake.getIntakeDemand())
                .getEntry();
        mIntakeDeployed = INTAKE_TAB
                .add("Intake Deployed", mIntake.getIsDeployed())
                .getEntry();

        /* INDEXER */
        mIndexerState = INDEXER_TAB
            .add("Indexer State", "N/A")
            .getEntry();
        mTopBeamBreak = INDEXER_TAB
            .add("Top Beam Break", false)
            .getEntry();
        mBottomBeamBreak = INDEXER_TAB
            .add("Bottom Beam Break", false)
            .getEntry();
        mBallCount = INDEXER_TAB
            .add("Ball Count", 0.0)
            .getEntry();

        mTunnelDemand = INDEXER_TAB
            .add("Tunnel Demand", 0.0)
            .getEntry();
        mTunnelVoltage = INDEXER_TAB
            .add("Tunnel Voltage", 0.0)
            .getEntry();
        mTunnelCurrent = INDEXER_TAB
            .add("Tunnel Current", 0.0)
            .getEntry();
        mTunnelVelocity = INDEXER_TAB
            .add("Tunnel Velocity", 0.0)
            .getEntry();

        mTriggerDemand = INDEXER_TAB
            .add("Trigger Demand", 0.0)
            .getEntry();
        mTriggerVoltage = INDEXER_TAB
            .add("Trigger Voltage", 0.0)
            .getEntry();
        mTriggerCurrent = INDEXER_TAB
            .add("Trigger Current", 0.0)
            .getEntry();
        mTriggerVelocity = INDEXER_TAB
            .add("Trigger Velocity", 0.0)
            .getEntry();
        
        /* SHOOTER */
        mFlywheelRPM = SHOOTER_TAB
                .add("Shooter RPM", 0.0)
                .withSize(2, 1)
                .getEntry();
        mAcceleratorRPM = SHOOTER_TAB
                .add("Accelerator RPM", 0.0)
                .withSize(2, 1)
                .getEntry();
        mFlywheelDemand = SHOOTER_TAB
                .add("Shooter Demand", 0.0)
                .withSize(2, 1)
                .getEntry();
        mAcceleratorDemand = SHOOTER_TAB
                .add("Accelerator Demand", 0.0)
                .withSize(2, 1)
                .getEntry();
        mShooterOpenLoop = SHOOTER_TAB
                .add("Shooter Open Loop", false)
                .withSize(2, 1)
                .getEntry();

        /* VISION */
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

        /* SUPERSTRUCTURE */
        // actions
        mIntaking = SUPERSTRUCTURE_TAB
            .add("Intaking", false)
            .withSize(2, 1)
            .getEntry();
        mOuttaking = SUPERSTRUCTURE_TAB
            .add("Outtaking", false)
            .withSize(2, 1)
            .getEntry();
        mPrepping = SUPERSTRUCTURE_TAB
            .add("Prepping", false)
            .withSize(2, 1)
            .getEntry();
        mShooting = SUPERSTRUCTURE_TAB
            .add("Shooting", false)
            .withSize(2, 1)
            .getEntry();
        mFender = SUPERSTRUCTURE_TAB
            .add("Fender Shot", false)
            .withSize(2, 1)
            .getEntry();

        // goals
        mIntakeGoal = SUPERSTRUCTURE_TAB
            .add("Intake Goal", "N/A")
            .withSize(2, 1)
            .getEntry();
        mIndexerGoal = SUPERSTRUCTURE_TAB
            .add("Indexer Goal", "N/A")
            .withSize(2, 1)
            .getEntry();
        mShooterGoal = SUPERSTRUCTURE_TAB
            .add("Shooter Goal", 0.0)
            .withSize(2, 1)
            .getEntry();
        mHoodGoal = SUPERSTRUCTURE_TAB
            .add("Hood Goal", 0.0)
            .withSize(2, 1)
            .getEntry();

        // additional status vars
        mSpunUp = SUPERSTRUCTURE_TAB
            .add("Is Spun Up", false)
            .withSize(2, 1)
            .getEntry();
        mHasTarget = SUPERSTRUCTURE_TAB
            .add("Has Vision Target", false)
            .withSize(2, 1)
            .getEntry();
        mIsAimed = SUPERSTRUCTURE_TAB
            .add("Is Vision Aimed", false)
            .withSize(2, 1)
            .getEntry();
        
        /* MANUAL PARAMS */
        mManualShooterRPM = MANUAL_PARAMS
            .add("Manual Shooter Goal", 0.0)
            .withSize(2, 1)
            .getEntry();
        mManualHoodAngle = MANUAL_PARAMS
            .add("Manual Hood Goal", 0.0)
            .withSize(2, 1)
            .getEntry();
        mShootingSetpointsEnableToggle = MANUAL_PARAMS
            .add("Apply Manual Shooting Params", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(2, 1)
            .getEntry();
    }
    

    public void update() {
        
        
        /* SWERVE */

        /*
        //  Only uncomment cancoder update when redoing cancoder offsets for modules
        // Update cancoders at a slower period to avoid stale can frames
        double dt = Timer.getFPGATimestamp();
        if (dt > lastCancoderUpdate + 0.1) {
            for (int i = 0; i < mSwerveCancoders.length; i++) {
                mSwerveCancoders[i].setDouble(truncate(mSwerveModules[i].getCanCoder().getDegrees()));
            }
            lastCancoderUpdate = dt;
        }
        */
        
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

        
        /* INTAKE */
        mIntakeState.setString(mIntake.getState().toString());
        mIntakeVoltage.setDouble(mIntake.getIntakeVoltage());
        mIntakeDemand.setDouble(mIntake.getIntakeDemand());
        mIntakeDeployed.setBoolean(mIntake.getWantDeploy());
        mIntakeCurrent.setDouble(mIntake.getIntakeCurrent());

        /* INDEXER */
        mIndexerState.setString(mIndexer.getState().toString());
        mTopBeamBreak.setBoolean(mIndexer.getTopBeamBreak());
        mBottomBeamBreak.setBoolean(mIndexer.getBottomBeamBreak());
        mBallCount.setDouble(mIndexer.getBallCount());

        mTunnelDemand.setDouble(truncate(mIndexer.getTunnelDemand()));
        mTunnelCurrent.setDouble(truncate(mIndexer.getTunnelCurrent()));
        mTunnelVoltage.setDouble(truncate(mIndexer.getTunnelVoltage()));
        mTunnelVelocity.setDouble(mIndexer.getTunnelVelocity());

        mTriggerDemand.setDouble(truncate(mIndexer.getTriggerDemand()));
        mTriggerCurrent.setDouble(truncate(mIndexer.getTriggerCurrent()));
        mTriggerVoltage.setDouble(truncate(mIndexer.getTriggerVoltage()));
        mTriggerVelocity.setDouble(mIndexer.getTriggerVelocity());

        /* SHOOTER */
        mFlywheelRPM.setDouble(truncate(mShooter.getFlywheelRPM()));
        mAcceleratorRPM.setDouble(truncate(mShooter.getAcceleratorRPM()));
        mShooterOpenLoop.setBoolean(mShooter.getIsOpenLoop());
        mFlywheelDemand.setDouble(truncate(mShooter.getFlywheelDemand()));
        mAcceleratorDemand.setDouble(truncate(mShooter.getAcceleratorDemand()));
        
        /* VISION */
        mSeesTarget.setBoolean(mLimelight.hasTarget());
        mLimelightOk.setBoolean(mLimelight.limelightOK());
        mLimelightLatency.setDouble(mLimelight.getLatency());
        mLimelightDt.setDouble(mLimelight.getDt());
        mLimelightTx.setDouble(mLimelight.getOffset()[0]);
        mLimelightTy.setDouble(mLimelight.getOffset()[1]);

        /* SUPERSTRUCTURE */
        // update actions statuses
        mIntaking.setBoolean(mSuperstructure.getIntaking());
        mOuttaking.setBoolean(mSuperstructure.getOuttaking());
        mPrepping.setBoolean(mSuperstructure.getPrepping());
        mShooting.setBoolean(mSuperstructure.getShooting());
        mFender.setBoolean(mSuperstructure.getWantsFender());

        // update superstructure goal statuses
        mIntakeGoal.setString(mSuperstructure.getIntakeGoal());
        mIndexerGoal.setString(mSuperstructure.getIndexerGoal());
        mShooterGoal.setDouble(mSuperstructure.getShooterGoal());
        mHoodGoal.setDouble(mSuperstructure.getHoodGoal());

        // update other status vars
        mSpunUp.setBoolean(mSuperstructure.isSpunUp());
        mHasTarget.setBoolean(mSuperstructure.hasTarget());
        mIsAimed.setBoolean(mSuperstructure.isAimed());

        /* MANUAL PARAMS (for shooting) */
        if(mShootingSetpointsEnableToggle.getValue().getBoolean()) {
            mSuperstructure.setShootingParameters(mManualShooterRPM.getDouble(0.0), mManualHoodAngle.getDouble(0.0));
        }
    }

    /* Truncates number to 2 decimal places for cleaner numbers */
    private double truncate(double number) {
        return Math.floor(number * 100) / 100;
    }
}
 