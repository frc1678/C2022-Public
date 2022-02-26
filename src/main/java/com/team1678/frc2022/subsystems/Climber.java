package com.team1678.frc2022.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {

    private final TalonFX mClimberRight;
    private final TalonFX mClimberLeft;

    public boolean mHomed;

    private static Climber mInstance;

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    public RightControlState mRightControlState = RightControlState.OPEN_LOOP;
    public LeftControlState mLeftControlState = LeftControlState.OPEN_LOOP;

    private boolean mExtendRightArm = false;
    private boolean mPartialExtendRightArm = false;
    private boolean mExtendLeftArm = false;
    private boolean mPartialExtendLeftArm = false;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private Climber() {
        mClimberRight = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_RIGHT_ID);
        mClimberLeft = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_LEFT_ID);

        // for right motor
        mClimberRight.set(ControlMode.PercentOutput, 0);
        mClimberRight.setInverted(false);
        mClimberRight.setSelectedSensorPosition(0.0);

        mClimberRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mClimberRight.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mClimberRight.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mClimberRight.config_kP(0, 0.6);
        mClimberRight.config_kI(0, 0);
        mClimberRight.config_kD(0, 0);
        mClimberRight.config_kF(0, 0.077);

        mClimberRight.setNeutralMode(NeutralMode.Brake);

        mClimberRight.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mClimberRight.enableVoltageCompensation(true);

        // for left motor
        mClimberLeft.set(ControlMode.PercentOutput, 0);
        mClimberLeft.setInverted(true);
        mClimberLeft.setSelectedSensorPosition(0.0);

        mClimberLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mClimberLeft.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mClimberLeft.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mClimberLeft.config_kP(0, 0.6);
        mClimberLeft.config_kI(0, 0);
        mClimberLeft.config_kD(0, 0);
        mClimberLeft.config_kF(0, 0.077);

        mClimberLeft.setNeutralMode(NeutralMode.Brake);

        mClimberLeft.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mClimberLeft.enableVoltageCompensation(true);

    }

    public static synchronized Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        } 
        return mInstance;
    }

    private void zeroEncoders() {
        mClimberLeft.setSelectedSensorPosition(0.0);
        mClimberRight.setSelectedSensorPosition(0.0);
        mHomed = true;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.climber_voltage_right = mClimberRight.getMotorOutputVoltage();
        mPeriodicIO.climber_stator_current_right = mClimberRight.getStatorCurrent();
        mPeriodicIO.climber_motor_velocity_right = mClimberRight.getSelectedSensorVelocity();
        mPeriodicIO.climber_motor_position_right = mClimberRight.getSelectedSensorPosition();
        
        mPeriodicIO.climber_voltage_left = mClimberLeft.getMotorOutputVoltage();
        mPeriodicIO.climber_stator_current_left = mClimberLeft.getStatorCurrent();
        mPeriodicIO.climber_motor_velocity_left = mClimberLeft.getSelectedSensorVelocity();
        mPeriodicIO.climber_motor_position_left = mClimberLeft.getSelectedSensorPosition();

        // send log data
        SendLog();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (mRightControlState) {
            case OPEN_LOOP:
                mClimberRight.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand_right / 12.0);
                break;
            case MOTION_MAGIC:
                mClimberRight.set(ControlMode.MotionMagic, mPeriodicIO.climber_demand_right);
                break;
            default:
                mClimberRight.set(ControlMode.MotionMagic,0.0);
                break;
        }

        switch (mLeftControlState) {
            case OPEN_LOOP:
                mClimberLeft.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand_left / 12.0);
                break;
            case MOTION_MAGIC:
                mClimberLeft.set(ControlMode.MotionMagic, mPeriodicIO.climber_demand_left);
                break;
            default:
                mClimberLeft.set(ControlMode.MotionMagic, 0.0);
                break;
        }

    }

    public synchronized void setBrakeMode(boolean brake) {
        mClimberRight.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
        mClimberLeft.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized void resetClimberPosition() {
        mClimberRight.setSelectedSensorPosition(0.0);
        mClimberLeft.setSelectedSensorPosition(0.0);
    }

    public void setRightClimberOpenLoop(double wantedDemand) {
        if (mRightControlState != RightControlState.OPEN_LOOP) {
            mRightControlState = RightControlState.OPEN_LOOP;
        }

        mPeriodicIO.climber_demand_right = (wantedDemand > 12 ? 12 : wantedDemand);

    }

    public void setLeftClimberOpenLoop(double wantedDemand) {
        if (mLeftControlState != LeftControlState.OPEN_LOOP) {
            mLeftControlState = LeftControlState.OPEN_LOOP;
        }

        mPeriodicIO.climber_demand_left = (wantedDemand > 12 ? 12 : wantedDemand);

    }

    public void setRightClimberPosition(double wantedPositionTicks) {
        if (mRightControlState != RightControlState.MOTION_MAGIC) {
            mRightControlState = RightControlState.MOTION_MAGIC;
        }
        
        mPeriodicIO.climber_demand_right = wantedPositionTicks;
    }

    public void setLeftClimberPosition(double wantedPositionTicks) {
        if (mLeftControlState != LeftControlState.MOTION_MAGIC) {
            mLeftControlState = LeftControlState.MOTION_MAGIC;
        }

        mPeriodicIO.climber_demand_left = wantedPositionTicks;
    }

    public void setRightClimberPositionDelta(double wantedPositionDelta) {
        if (mRightControlState != RightControlState.MOTION_MAGIC) {
            mRightControlState = RightControlState.MOTION_MAGIC;
            mPeriodicIO.climber_demand_right = mPeriodicIO.climber_motor_position_right;
        }

        mPeriodicIO.climber_demand_right = mPeriodicIO.climber_demand_right + wantedPositionDelta;
    }

    public void setLeftClimberPositionDelta(double wantedPositionDelta) {
        if (mLeftControlState != LeftControlState.MOTION_MAGIC) {
            mLeftControlState = LeftControlState.MOTION_MAGIC;
            mPeriodicIO.climber_demand_left = mPeriodicIO.climber_motor_position_left;
        }

        mPeriodicIO.climber_demand_left = mPeriodicIO.climber_demand_left + wantedPositionDelta;
    }

    /*** CLIMB METHODS
     * 
     * 1. always start climbing using setExtendForClimb() to prep for climb action
     * 
     * 2. use setClimbMidBar() when only climbing to second bar
     * 
     * 3. use labeled sequence of traversal climb steps for traversal climb
     *    - setClimbMidBarAndExtend() to climb mid bar with right arm and extend left arm
     *    - setClimbHighBarAndExtend() to climb high bar with left arm and extend right arm
     *    - setClimbTraversalBar() to climb a partial distance with right arm on traversal bar
     * 
     * IF NOT DOING ANY OF THE ABOVE: set
     *
     * */

    // extend right arm to start climbing
    public void setExtendForClimb() {
        setRightClimberPosition(Constants.ClimberConstants.kRightTravelDistance);
    }

    // use when only climbing to mid bar
    public void setClimbMidBar() {
        setRightClimberPosition(Constants.ClimberConstants.kSafetyMinimum);
    }

    // second step for traversal
    public void setClimbMidBarAndExtend() {
        setLeftClimberPosition(Constants.ClimberConstants.kLeftPartialTravelDistance);
        setRightClimberPosition(Constants.ClimberConstants.kSafetyMinimum);
    }
    // third step for traversal
    public void setHighBarExtend() {
        setLeftClimberPosition(Constants.ClimberConstants.kLeftTravelDistance);
    }
    // third step for traversal
    public void setClimbHighBarAndExtend() {
        setLeftClimberPosition(Constants.ClimberConstants.kSafetyMinimum);
        setRightClimberPosition(Constants.ClimberConstants.kRightPartialTravelDistance);
    }
    public void setTraversalBarExtend() {
        setRightClimberPosition(Constants.ClimberConstants.kRightTravelDistance);
    }
    // final step for traversal
    public void setClimbTraversalBar() {
        setRightClimberPosition(Constants.ClimberConstants.kRightPartialTravelDistance);
    }

    public void setClimberNone() {
        setRightClimberPosition(10); // ticks
        setLeftClimberPosition(10); // ticks

    }

    // hold current position on arm
    public void holdCurrentPosition() {
        if (mPeriodicIO.climber_stator_current_left > Constants.ClimberConstants.kStatorCurrentLimit) {
            setLeftClimberPosition(mPeriodicIO.climber_motor_position_left);
        }

        if (mPeriodicIO.climber_stator_current_right > Constants.ClimberConstants.kStatorCurrentLimit) {
            setRightClimberPosition(mPeriodicIO.climber_motor_position_right);
        }
    }

    public enum RightControlState {
        HOMING,
        OPEN_LOOP,
        MOTION_MAGIC
    }

    public enum LeftControlState {
        HOMING,
        OPEN_LOOP,
        MOTION_MAGIC
    }
    
    public void stop() {
        mClimberRight.set(ControlMode.PercentOutput, 0.0);
        mClimberLeft.set(ControlMode.PercentOutput, 0.0);
    }

    public double getClimberVelocityRight() {
        return mPeriodicIO.climber_motor_velocity_right;
    }

    public double getClimberVelocityLeft() {
        return mPeriodicIO.climber_motor_velocity_left;
    }

    public double getClimberDemandRight() {
        return mPeriodicIO.climber_demand_right;
    }

    public double getClimberDemandLeft() {
        return mPeriodicIO.climber_demand_left;
    }

    public double getClimberPositionRight() {
        return mPeriodicIO.climber_motor_position_right;
    }

    public double getClimberPositionLeft() {
        return mPeriodicIO.climber_motor_position_left;
    }

    public double getClimberCurrentRight() {
        return mPeriodicIO.climber_stator_current_right;
    }

    public double getClimberCurrentLeft() {
        return mPeriodicIO.climber_stator_current_left;
    }

    public boolean getHomed() {
        return mHomed;
    }

    public RightControlState getRightControlState() {
        return mRightControlState;
    }

    public LeftControlState getLeftControlState() {
        return mLeftControlState;
    }

    public boolean checkSystem() {
        return true;
    }

    public void setClimberDemandRight(double demand) {
        mPeriodicIO.climber_demand_right = demand;
    }

    public void setClimberDemandLeft(double demand) {
        mPeriodicIO.climber_demand_left = demand;
    }

    public void toggleExtendRightArm() {
        mExtendRightArm = !mExtendRightArm;
    }

    public void togglePartialExtendRightArm() {
        mPartialExtendRightArm = !mPartialExtendRightArm;
    }

    public void toggleExtendLeftArm() {
        mExtendLeftArm = !mExtendLeftArm;
    }

    public void togglePartialExtendLeftArm() {
        mPartialExtendLeftArm = !mPartialExtendLeftArm;
    }

    public boolean getExtendRightArm() {
        return mExtendRightArm;
    }

    public boolean getExtendLeftArm() {
        return mExtendLeftArm;
    }

    public boolean getPartialExtendRightArm() {
        return mPartialExtendRightArm;
    }

    public boolean getPartialExtendLeftArm() {
        return mPartialExtendLeftArm;
    }

    public boolean hasEmergency = false;

    public void outputTelemetry() {
        SmartDashboard.putNumber("Climber Demand Right", mPeriodicIO.climber_demand_right);
        SmartDashboard.putNumber("Climber Voltage Right", mPeriodicIO.climber_voltage_right);
        SmartDashboard.putNumber("Climber Current Right", mPeriodicIO.climber_stator_current_right);
        SmartDashboard.putBoolean("Extend Right Climber", getExtendRightArm());
        SmartDashboard.putBoolean("Partial Extend Right Climber", getPartialExtendRightArm());

        SmartDashboard.putNumber("Climber Demand Left", mPeriodicIO.climber_demand_left);
        SmartDashboard.putNumber("Climber Voltage Left", mPeriodicIO.climber_voltage_left);
        SmartDashboard.putNumber("Climber Current Left", mPeriodicIO.climber_stator_current_left);
        SmartDashboard.putBoolean("Extend Left Climber", getExtendLeftArm());
        SmartDashboard.putBoolean("Partial Extend Left Climber", getPartialExtendLeftArm());
    }

    public static class PeriodicIO {
        /* Inputs */
        public double climber_voltage_right;
        public double climber_stator_current_right;
        public double climber_motor_position_right;
        public double climber_motor_velocity_right;
        
        public double climber_voltage_left;
        public double climber_stator_current_left;
        public double climber_motor_position_left;
        public double climber_motor_velocity_left;

        /* Outputs */
        public double climber_demand_right;
        public double climber_demand_left;
    }

    // logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "CLIMBER_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");
        headers.add("climber_stator_current_right");
        headers.add("climber_motor_position_right");
        headers.add("climber_voltage_left");
        headers.add("climber_motor_velocity_right");
        headers.add("climber_stator_current_left");
        headers.add("climber_demand_right");
        headers.add("climber_motor_position_left");
        headers.add("climber_demand_left");
        headers.add("climber_voltage_right");
        headers.add("climber_motor_velocity_left");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());
        items.add(mPeriodicIO.climber_stator_current_right);
        items.add(mPeriodicIO.climber_motor_position_right);
        items.add(mPeriodicIO.climber_voltage_left);
        items.add(mPeriodicIO.climber_motor_velocity_right);
        items.add(mPeriodicIO.climber_stator_current_left);
        items.add(mPeriodicIO.climber_demand_right);
        items.add(mPeriodicIO.climber_motor_position_left);
        items.add(mPeriodicIO.climber_demand_left);
        items.add(mPeriodicIO.climber_voltage_right);
        items.add(mPeriodicIO.climber_motor_velocity_left);

        // send data to logging storage
        mStorage.addData(items);
    }
} 
