package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {

    private final TalonFX mClimberRight;
    private final TalonFX mClimberLeft;

    public boolean mHomed;

    private static Climber mInstance;

    public ControlState mControlState = ControlState.OPEN_LOOP;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private Climber() {
        mClimberRight = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_RIGHT_ID);
        mClimberLeft = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_LEFT_ID);

        //For Right motor
        mClimberRight.set(ControlMode.PercentOutput, 0);
        mClimberRight.setInverted(false);

        mClimberRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mClimberRight.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mClimberRight.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mClimberRight.config_kP(0, 0.5);
        mClimberRight.config_kI(0, 0);
        mClimberRight.config_kD(0, 0);
        mClimberRight.config_kF(0, 0.05);

        mClimberRight.setNeutralMode(NeutralMode.Brake);

        mClimberRight.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mClimberRight.enableVoltageCompensation(true);

        //For left motor
        mClimberLeft.set(ControlMode.PercentOutput, 0);
        mClimberLeft.setInverted(false);

        mClimberLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mClimberLeft.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mClimberLeft.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mClimberLeft.config_kP(0, 0.5);
        mClimberLeft.config_kI(0, 0);
        mClimberLeft.config_kD(0, 0);
        mClimberLeft.config_kF(0, 0.05);

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
        mPeriodicIO.climber_stator_current_right = mClimberRight.getStatorCurrent();
        mPeriodicIO.climber_motor_velocity_right = mClimberRight.getSelectedSensorVelocity();
        mPeriodicIO.climber_motor_position_right = mClimberRight.getSelectedSensorPosition();
        
        mPeriodicIO.climber_stator_current_left = mClimberLeft.getStatorCurrent();
        mPeriodicIO.climber_motor_velocity_left = mClimberLeft.getSelectedSensorVelocity();
        mPeriodicIO.climber_motor_position_left = mClimberLeft.getSelectedSensorPosition();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (mControlState) {
            case OPEN_LOOP:
                /* If holding position, set to position control to avoid sag */
                // if (mPeriodicIO.climber_demand == 0.0) {
                    // setClimberPositionDelta(0.0);
                // } else {
                    mClimberRight.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand_right / 12.0);
                    mClimberLeft.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand_left / 12.0);
                    // mClimberSlave.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand / 12.0);
                //}
                break;
            case MOTION_MAGIC:
                mClimberRight.set(ControlMode.MotionMagic, mPeriodicIO.climber_demand_right);
                mClimberLeft.set(ControlMode.MotionMagic, mPeriodicIO.climber_demand_left);
                break;
            default:
                mClimberRight.set(ControlMode.MotionMagic,0.0);
                mClimberLeft.set(ControlMode.MotionMagic, 0.0);
                break;
        }

    }

    public void setClimberOpenLoop(double wantedDemand) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }
        mPeriodicIO.climber_demand_right = (wantedDemand > 12 ? 12 : wantedDemand);
        mPeriodicIO.climber_demand_left = (wantedDemand > 12 ? 12 : wantedDemand);

    }

    public void setClimberPosition(double wantedPositionTicks) {
        if (mControlState != ControlState.MOTION_MAGIC) {
            mControlState = ControlState.MOTION_MAGIC;
        }
        mPeriodicIO.climber_demand_right = wantedPositionTicks;
        mPeriodicIO.climber_demand_left = wantedPositionTicks;
    }

    public void setClimberPositionDelta(double wantedPositionDelta) {
        if (mControlState != ControlState.MOTION_MAGIC) {
            mControlState = ControlState.MOTION_MAGIC;
            mPeriodicIO.climber_demand_right = mPeriodicIO.climber_motor_position_right;
            mPeriodicIO.climber_demand_left = mPeriodicIO.climber_motor_position_left;
        }
        mPeriodicIO.climber_demand_right = mPeriodicIO.climber_demand_right + wantedPositionDelta;
        mPeriodicIO.climber_demand_left = mPeriodicIO.climber_demand_left + wantedPositionDelta;
    }

    public enum ControlState {
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

    public ControlState getControlState() {
        return mControlState;
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

    public boolean hasEmergency = false;

    public void outputTelemetry() {
    }

    public static class PeriodicIO {
        /* Inputs */
        public double climber_stator_current_right;
        public double climber_motor_position_right;
        public double climber_motor_velocity_right;
        
        public double climber_stator_current_left;
        public double climber_motor_position_left;
        public double climber_motor_velocity_left;


        /* Outputs */
        public double climber_demand_right;
        public double climber_demand_left;
    }
} 
