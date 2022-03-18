package com.team1678.frc2022;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import com.team1678.frc2022.subsystems.ColorSensor;

import com.team1678.frc2022.subsystems.Climber;
import com.team1678.frc2022.subsystems.Indexer;
import com.team1678.frc2022.subsystems.Intake;
import com.team1678.frc2022.subsystems.LEDs;
import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.Shooter;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;

import com.team1678.frc2022.subsystems.Trigger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private final LEDs mLEDs;
    private final Shooter mShooter;
    private final Trigger mTrigger;
    private final Indexer mIndexer;
    private final Climber mClimber;
    private final Superstructure mSuperstructure;
    private final ColorSensor mColorSensor;

    /* Status Variables */
    private double lastCancoderUpdate = 0.0;

    /* Tabs */
    private ShuffleboardTab OPERATOR_TAB;
    private ShuffleboardTab VISION_TAB;
    private ShuffleboardTab SWERVE_TAB;
    private ShuffleboardTab PID_TAB;
    private ShuffleboardTab LED_TAB;
    private ShuffleboardTab INTAKE_TAB;
    private ShuffleboardTab SHOOTER_TAB;
    private ShuffleboardTab INDEXER_TAB;
    private ShuffleboardTab CLIMBER_TAB;
    private ShuffleboardTab SUPERSTRUCTURE_TAB;
    private ShuffleboardTab MANUAL_PARAMS;
    private ShuffleboardTab COLOR_SENSOR;   

    /*** ENTRIES ***/
    
    /* CANdle */
    private final NetworkTableEntry mTopLEDState;
    private final NetworkTableEntry mBottomLEDState;
    
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
    private final NetworkTableEntry mIntakeState;
    private final NetworkTableEntry mIntakeRollerCurrent;
    private final NetworkTableEntry mIntakeRollerVoltage;
    private final NetworkTableEntry mIntakeRollerDemand;
    private final NetworkTableEntry mIntakeDeployCurrent;
    private final NetworkTableEntry mIntakeDeployVoltage;
    private final NetworkTableEntry mIntakeDeployDemand;

    // private final NetworkTableEntry mFlywheelManualPIDToggle;
    // private final NetworkTableEntry mFlywheelP;
    // private final NetworkTableEntry mFlywheelI;
    // private final NetworkTableEntry mFlywheelD;
    // private final NetworkTableEntry mFlywheelF;

    /* SHOOTER */
    private final NetworkTableEntry mFlywheelRPM;
    private final NetworkTableEntry mShooterOpenLoop;
    private final NetworkTableEntry mFlywheelDemand;

    private final NetworkTableEntry mTriggerVelocity;
    private final NetworkTableEntry mTriggerCurrent;
    private final NetworkTableEntry mTriggerDemand;
    private final NetworkTableEntry mTriggerVoltage;
    /* VISION */
    private final NetworkTableEntry mSeesTarget;
    private final NetworkTableEntry mLimelightOk;
    private final NetworkTableEntry mLimelightLatency;
    private final NetworkTableEntry mLimelightDt;
    private final NetworkTableEntry mLimelightTx;
    private final NetworkTableEntry mLimelightTy;

    /* INDEXER */
    private final NetworkTableEntry mEjectorCurrent;
    private final NetworkTableEntry mEjectorDemand;
    private final NetworkTableEntry mEjectorVoltage;

    private final NetworkTableEntry mTunnelCurrent;
    private final NetworkTableEntry mTunnelDemand;
    private final NetworkTableEntry mTunnelVoltage;



    private final NetworkTableEntry mIndexerState;

    private final NetworkTableEntry mBallCount;

    private final NetworkTableEntry mTopBeamBreak;
    private final NetworkTableEntry mBottomBeamBreak;

    /* CLIMBER */
    private final NetworkTableEntry mClimberVelocityRight;
    private final NetworkTableEntry mClimberVelocityLeft;

    private final NetworkTableEntry mClimberDemandRight;
    private final NetworkTableEntry mClimberDemandLeft;

    private final NetworkTableEntry mClimberPositionRight;
    private final NetworkTableEntry mClimberPositionLeft;

    private final NetworkTableEntry mClimberCurrentRight;
    private final NetworkTableEntry mClimberCurrentLeft;

    private final NetworkTableEntry mClimberHomed;

    private final NetworkTableEntry mClimberLeftControlState;
    private final NetworkTableEntry mClimberRightControlState;

    private final NetworkTableEntry mInClimbMode;
    private final NetworkTableEntry mOpenLoopClimbControl;

    /* SUPERSTRUCTURE */
    // actions
    private final NetworkTableEntry mIntaking;
    private final NetworkTableEntry mReversing;
    private final NetworkTableEntry mRejecting;
    private final NetworkTableEntry mEjecting;
    private final NetworkTableEntry mPrepping;
    private final NetworkTableEntry mShooting;
    private final NetworkTableEntry mFenderShot;
    private final NetworkTableEntry mSpitShot;

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

    /* COLOR SENSOR */
    private final NetworkTableEntry mSensor0;
    private final NetworkTableEntry mRValue;
    private final NetworkTableEntry mGValue;
    private final NetworkTableEntry mBValue;
    private final NetworkTableEntry mAllianceColor;
    private final NetworkTableEntry mMatchedColor;
    private final NetworkTableEntry mForwardBreak;

    private final NetworkTableEntry mHasBall;
    private final NetworkTableEntry mEject;

    private final NetworkTableEntry mTimestamp;
  
    /* Operator */
    private final NetworkTableEntry mOperatorShooting;
    private final NetworkTableEntry mOperatorSpunup;
    private final NetworkTableEntry mOperatorFender;
    private final NetworkTableEntry mOperatorSpit;
    private final NetworkTableEntry mOperatorVisionAimed;
    private final NetworkTableEntry mOperatorClimbMode;
    private final NetworkTableEntry mOperatorAutoClimb;

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        /* Get Subsystems */
        mSwerve = Swerve.getInstance();
        mSwerveModules = Swerve.getInstance().mSwerveMods;
        mIntake = Intake.getInstance();
        mIndexer = Indexer.getInstance();
        mLEDs = LEDs.getInstance();
        mClimber = Climber.getInstance();
        mShooter = Shooter.getInstance();
        mTrigger = Trigger.getInstance();
        mLimelight = Limelight.getInstance();
        mSuperstructure = Superstructure.getInstance();
        mColorSensor = ColorSensor.getInstance();

        /* Get Tabs */
        OPERATOR_TAB = Shuffleboard.getTab("OPERATOR");
        SWERVE_TAB = Shuffleboard.getTab("Swerve");
        PID_TAB = Shuffleboard.getTab("Module PID");
        LED_TAB = Shuffleboard.getTab("LEDs");
        INTAKE_TAB = Shuffleboard.getTab("Intake");
        INDEXER_TAB = Shuffleboard.getTab("Indexer");
        CLIMBER_TAB = Shuffleboard.getTab("Climber");
        SHOOTER_TAB = Shuffleboard.getTab("Shooter");
        VISION_TAB = Shuffleboard.getTab("Vision");
        SUPERSTRUCTURE_TAB = Shuffleboard.getTab("Superstructure");
        MANUAL_PARAMS = Shuffleboard.getTab("Manual Params");
        COLOR_SENSOR = Shuffleboard.getTab("Color Sensor");
        
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

        /* CANdle */
        mTopLEDState = LED_TAB
            .add("Top LEDs State", "N/A")
            .withSize(2, 1)
            .getEntry();

        mBottomLEDState = LED_TAB
            .add("Bottom LEDs State", "N/A")
            .withSize(2, 1)
            .getEntry();
        
        /* INTAKE */
        mIntakeState = INTAKE_TAB
            .add("Intake State", "N/A")
            .getEntry();
        mIntakeRollerCurrent = INTAKE_TAB
            .add("Roller Current", 0.0)
            .getEntry();
        mIntakeRollerVoltage = INTAKE_TAB
            .add("Roller Voltage", 0.0)
            .getEntry();
        mIntakeRollerDemand = INTAKE_TAB
            .add("Roller Demand", 0.0)
            .getEntry();
        mIntakeDeployCurrent = INTAKE_TAB
            .add("Deploy Current", 0.0)
            .getEntry();
        mIntakeDeployVoltage = INTAKE_TAB
            .add("Deploy Voltage", 0.0)
            .getEntry();
        mIntakeDeployDemand= INTAKE_TAB
            .add("Deploy Demand", 0.0)
            .getEntry();
        

        /* INDEXER */
        mIndexerState = INDEXER_TAB
            .add("Indexer State", "N/A")
            .getEntry();
        mEjectorCurrent = INDEXER_TAB
            .add("Outtake Current", 0.0)
            .getEntry();
        mEjectorDemand = INDEXER_TAB
            .add("Outtake Demand", 0.0)
            .getEntry();
        mEjectorVoltage = INDEXER_TAB
            .add("Outtake Voltage", 0.0)
            .getEntry(); 
        mTunnelCurrent = INDEXER_TAB
            .add("Indexer Current", 0.0)
            .getEntry();
        mTunnelDemand = INDEXER_TAB
            .add("Indexer Demand", 0.0)
            .getEntry();
        mTunnelVoltage = INDEXER_TAB
            .add("Indexer Voltage", 0.0)
            .getEntry();
        mTopBeamBreak = INDEXER_TAB
            .add("Top Beam Break Triggered", false)
            .getEntry();
        mBottomBeamBreak = INDEXER_TAB
            .add("Bottom Beam Break Triggered", false)
            .getEntry();
        mBallCount = INDEXER_TAB
            .add("Ball Count", 0.0)
            .getEntry();

        /* CLIMBER */
        mInClimbMode = CLIMBER_TAB
            .add("Climb Mode", false)
            .getEntry();
        mOpenLoopClimbControl = CLIMBER_TAB
            .add("Open Loop Climb Control", false)
            .getEntry();
        mClimberLeftControlState = CLIMBER_TAB
            .add("Climber Left Control State", mClimber.mLeftControlState.toString())
            .getEntry();
        mClimberRightControlState = CLIMBER_TAB
            .add("Climber Right Control State", mClimber.mRightControlState.toString())
            .getEntry();
        mClimberVelocityRight = CLIMBER_TAB
            .add("Right Climber Velocity", 0.0)
            .getEntry();
        mClimberVelocityLeft = CLIMBER_TAB
            .add("Left Climber Velocity", 0.0)
            .getEntry();
        mClimberDemandRight = CLIMBER_TAB
            .add("Right Climber Demand", 0.0)
            .getEntry();
        mClimberDemandLeft = CLIMBER_TAB
            .add("Left Climber Demand", 0.0)
            .getEntry();
        mClimberPositionRight = CLIMBER_TAB
            .add("Right Climber Position", 0.0)
            .getEntry();
        mClimberPositionLeft = CLIMBER_TAB
            .add("Left Climber Position", 0.0)
            .getEntry();
        mClimberCurrentRight = CLIMBER_TAB
            .add("Right Climber Current", 0.0)
            .getEntry();
        mClimberCurrentLeft = CLIMBER_TAB 
            .add("Left Climber Current", 0.0)
            .getEntry();
        mClimberHomed = CLIMBER_TAB
            .add("Climber is Homed", false)
            .getEntry();

        /* SHOOTER */
        mFlywheelRPM = SHOOTER_TAB
                .add("Shooter RPM", 0.0)
                .withSize(2, 1)
                .getEntry();
        mFlywheelDemand = SHOOTER_TAB
                .add("Shooter Demand", 0.0)
                .withSize(2, 1)
                .getEntry();
        mTriggerCurrent = INDEXER_TAB
                .add("Trigger Current", 0.0)
                .withSize(2, 1)
                .getEntry();
        mTriggerDemand = INDEXER_TAB
                .add("Trigger Demand", 0.0)
                .withSize(2, 1)
                .getEntry();
        mTriggerVoltage = INDEXER_TAB
                .add("Trigger Voltage", 0.0)
                .withSize(2, 1)
                .getEntry();
        mTriggerVelocity = INDEXER_TAB
                .add("Trigger Velocity", 0.0)
                .withSize(2, 1)
                .getEntry();
        mShooterOpenLoop = SHOOTER_TAB
                .add("Shooter Open Loop", false)
                .withSize(2, 1)
                .getEntry();
        
        /* COLOR SENSOR */
        mSensor0 = COLOR_SENSOR
            .add("Is Sensor 0 Connected", false)
            .getEntry();
        mRValue = COLOR_SENSOR
            .add("Detected R Value", 0.0)
            .getEntry();
        mGValue = COLOR_SENSOR
            .add("Detected G Value", 0.0)
            .getEntry();
        mBValue = COLOR_SENSOR
            .add("Detected B Value", 0.0)
            .getEntry();
        mAllianceColor = COLOR_SENSOR
            .add("Alliance Color", "N/A")
            .getEntry();
        mMatchedColor = COLOR_SENSOR
            .add("Matched Color", "N/A")
            .getEntry();
        mForwardBreak = COLOR_SENSOR
            .add("Read Distance", 0.0)
            .getEntry();
            
        mHasBall = COLOR_SENSOR
            .add("Has Ball", false)
            .getEntry();
        mEject = COLOR_SENSOR
            .add("Eject", false)
            .getEntry();

        mTimestamp = COLOR_SENSOR
            .add("Timestamp", 0.0)
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
            .withSize(1, 1)
            .getEntry();
        mReversing = SUPERSTRUCTURE_TAB
            .add("Reversing", false)
            .withSize(1, 1)
            .getEntry();
        mRejecting = SUPERSTRUCTURE_TAB
            .add("Rejecting", false)
            .withSize(1, 1)
            .getEntry();
        mEjecting = SUPERSTRUCTURE_TAB
            .add("Ejecting", false)
            .withSize(1, 1)
            .getEntry();
        mPrepping = SUPERSTRUCTURE_TAB
            .add("Prepping", false)
            .withSize(1, 1)
            .getEntry();
        mShooting = SUPERSTRUCTURE_TAB
            .add("Shooting", false)
            .withSize(1, 1)
            .getEntry();
        mFenderShot = SUPERSTRUCTURE_TAB
            .add("Fender Shot", false)
            .withSize(1, 1)
            .getEntry();
        mSpitShot = SUPERSTRUCTURE_TAB
            .add("Spit Shot", false)
            .withSize(1, 1)
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

        /* Operator */
        mOperatorShooting = OPERATOR_TAB
                .add("Shooting", false)
                .withSize(3, 2)
                .withPosition(2, 1)
                .getEntry();
        mOperatorSpunup = OPERATOR_TAB
                .add("Spun Up", false)
                .withSize(3,2)
                .withPosition(5, 1)
                .getEntry();
        mOperatorSpit = OPERATOR_TAB
                .add("Spitting", false)
                .withSize(3, 1)
                .withPosition(2, 0)
                .getEntry();
        mOperatorFender = OPERATOR_TAB
                .add("Fender", false)
                .withSize(3, 1)
                .withPosition(5, 0)
                .getEntry();
        mOperatorVisionAimed = OPERATOR_TAB
                .add("Vision Aimed", false)
                .withSize(6, 2)
                .withPosition(2, 3)
                .getEntry();
        mOperatorClimbMode = OPERATOR_TAB
                .add("Climb Mode", false)
                .withSize(2, 2)
                .withPosition(8, 0)
                .getEntry();
        mOperatorAutoClimb = OPERATOR_TAB
                .add("Auto Climbing", false)
                .withSize(2, 2)
                .withPosition(8, 2)
                .getEntry();
    }

    public void update() {

        /* OPERATOR */
        mOperatorShooting.setBoolean(mSuperstructure.getShooting());
        mOperatorSpunup.setBoolean(mSuperstructure.isSpunUp());
        mOperatorSpit.setBoolean(mSuperstructure.getWantsSpit());
        mOperatorFender.setBoolean(mSuperstructure.getWantsFender());
        mOperatorVisionAimed.setBoolean(mLimelight.isAimed());
        mOperatorClimbMode.setBoolean(mSuperstructure.getInClimbMode());
        mOperatorAutoClimb.setBoolean(mSuperstructure.isAutoClimb());

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
        
        
        for (int i = 0; i < mSwerveCancoders.length; i++) {
            mSwerveIntegrated[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getState().angle.getDegrees(), 0, 360)));
            mSwerveDrivePercent[i].setDouble(truncate(mSwerveModules[i].getState().speedMetersPerSecond));

            mModuleAngleCurrent[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getState().angle.getDegrees(), 0, 360)));
            mModuleAngleGoals[i].setDouble(truncate(MathUtil.inputModulus(mSwerveModules[i].getTargetAngle(), 0, 360)));

        }
        */

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

        mIntakeRollerCurrent.setDouble(mIntake.getRollerCurrent());
        mIntakeRollerVoltage.setDouble(mIntake.getRollerVoltage());
        mIntakeRollerDemand.setDouble(mIntake.getRollerDemand());

        mIntakeDeployCurrent.setDouble(mIntake.getDeployCurrent());
        mIntakeDeployVoltage.setDouble(mIntake.getDeployVoltage());
        mIntakeDeployDemand.setDouble(mIntake.getDeployDemand());
        
        /* SHOOTER */
        mFlywheelRPM.setDouble(truncate(mShooter.getFlywheelRPM()));
        mShooterOpenLoop.setBoolean(mShooter.getIsOpenLoop());
        mFlywheelDemand.setDouble(truncate(mShooter.getFlywheelDemand()));

        mTriggerCurrent.setDouble(mTrigger.getTriggerCurrent());
        mTriggerDemand.setDouble(mTrigger.getTriggerDemand());
        mTriggerVoltage.setDouble(mTrigger.getTriggerVoltage());
        mTriggerVelocity.setDouble(mTrigger.getTriggerVelocity());
        
        /* VISION */
        mSeesTarget.setBoolean(mLimelight.hasTarget());
        mLimelightOk.setBoolean(mLimelight.limelightOK());
        mLimelightLatency.setDouble(mLimelight.getLatency());
        mLimelightDt.setDouble(mLimelight.getDt());
        mLimelightTx.setDouble(mLimelight.getOffset()[0]);
        mLimelightTy.setDouble(mLimelight.getOffset()[1]);

        /* INDEXER */
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

        /* COLOR SENSOR */
        mSensor0.setBoolean(mColorSensor.getSensor0());
        mRValue.setDouble(mColorSensor.getDetectedRValue());
        mGValue.setDouble(mColorSensor.getDetectedGValue());
        mBValue.setDouble(mColorSensor.getDetectedBValue());
        mAllianceColor.setString(mColorSensor.getAllianceColor().toString());
        mMatchedColor.setString(mColorSensor.getMatchedColor().toString());
        mForwardBreak.setBoolean(mColorSensor.getFowrardBeamBreak());

        mHasBall.setBoolean(mColorSensor.hasBall());
        mEject.setBoolean(mColorSensor.wantsEject());

        mTimestamp.setDouble(mColorSensor.getTimestamp());

        /* CLIMBER */
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

        /* SUPERSTRUCTURE */
        // update actions statuses
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

        // Lights
        mTopLEDState.setString(mLEDs.getTopState().getName());
        mBottomLEDState.setString(mLEDs.getBottomState().getName());
    }

    /* Truncates number to 2 decimal places for cleaner numbers */
    private double truncate(double number) {
        return Math.floor(number * 100) / 100;
    }

    public ShuffleboardTab getOperatorTab() {
        return OPERATOR_TAB;
    }
}
 