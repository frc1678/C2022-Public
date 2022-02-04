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

    private final TalonFX mClimberMaster;
    private final TalonFX mClimberSlave;

    public final Solenoid mInitialReleaseClimberSolenoid;
    public final Solenoid mChopstickClimberBarSolenoid;
    public final Solenoid mHookClimberSolenoid;
    public boolean mHomed;

    private static Climber mInstance;

    public ControlState mControlState = ControlState.OPEN_LOOP;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private Climber() {
        mClimberMaster = TalonFXFactory.createDefaultTalon(Ports.CLIMBER_MASTER_ID);
        mClimberSlave = TalonFXFactory.createPermanentSlaveTalon(Ports.CLIMBER_SLAVE_ID, Ports.CLIMBER_MASTER_ID);

        mClimberMaster.set(ControlMode.PercentOutput, 0);
        mClimberMaster.setInverted(false);
        mClimberSlave.setInverted(true);

        mClimberMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mClimberMaster.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mClimberMaster.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mClimberMaster.config_kP(0, 0.5);
        mClimberMaster.config_kI(0, 0);
        mClimberMaster.config_kD(0, 0);
        mClimberMaster.config_kF(0, 0.05);

        mClimberMaster.setNeutralMode(NeutralMode.Brake);
        mClimberSlave.setNeutralMode(NeutralMode.Brake);

        mClimberMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mClimberMaster.enableVoltageCompensation(true);

        mChopstickClimberBarSolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.CLIMBER_CHOPSTICK_SOLENOID);
        mHookClimberSolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.CLIMBER_HOOK_RELEASE_SOLENOID);
        mInitialReleaseClimberSolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.CLIMBER_INITIAL_RELEASE_SOLENOID);

    }

    public static synchronized Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        } 
        return mInstance;
    }

    private void zeroEncoders() {
        mClimberMaster.setSelectedSensorPosition(0.0);
        mHomed = true;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.climber_stator_current = mClimberMaster.getStatorCurrent();
        mPeriodicIO.climber_motor_velocity = mClimberMaster.getSelectedSensorVelocity();
        mPeriodicIO.climber_motor_position = mClimberMaster.getSelectedSensorPosition();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (mControlState) {
            case OPEN_LOOP:
                /* If holding position, set to position control to avoid sag */
                // if (mPeriodicIO.climber_demand == 0.0) {
                    // setClimberPositionDelta(0.0);
                // } else {
                    mClimberMaster.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand / 12.0);
                    // mClimberSlave.set(ControlMode.PercentOutput, mPeriodicIO.climber_demand / 12.0);
                //}
                break;
            case MOTION_MAGIC:
                mClimberMaster.set(ControlMode.MotionMagic, mPeriodicIO.climber_demand);
                break;
            default:
                mClimberMaster.set(ControlMode.MotionMagic,0.0);
                break;
        }
        
        mInitialReleaseClimberSolenoid.set(mPeriodicIO.release_solenoid);
        mChopstickClimberBarSolenoid.set(mPeriodicIO.chopsticks_solenoid);
        mHookClimberSolenoid.set(mPeriodicIO.hook_solenoid);
    }

    public void setClimberOpenLoop(double wantedDemand) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }
        mPeriodicIO.climber_demand = (wantedDemand > 12 ? 12 : wantedDemand);
    }

    public void setClimberPosition(double wantedPositionTicks) {
        if (mControlState != ControlState.MOTION_MAGIC) {
            mControlState = ControlState.MOTION_MAGIC;
        }
        mPeriodicIO.climber_demand = wantedPositionTicks;
    }

    public void setClimberPositionDelta(double wantedPositionDelta) {
        if (mControlState != ControlState.MOTION_MAGIC) {
            mControlState = ControlState.MOTION_MAGIC;
            mPeriodicIO.climber_demand = mPeriodicIO.climber_motor_position;
        }
        mPeriodicIO.climber_demand = mPeriodicIO.climber_demand + wantedPositionDelta;
    }

    public enum ControlState {
        HOMING,
        OPEN_LOOP,
        MOTION_MAGIC
    }
    
    public void stop() {
        mClimberMaster.set(ControlMode.PercentOutput, 0.0);
    }

    public boolean getClimberSolenoidDeployed() {
       return mPeriodicIO.chopsticks_solenoid;
    }

    public double getClimberVelocity() {
        return mPeriodicIO.climber_motor_velocity;
    }

    public double getClimberDemand() {
        return mPeriodicIO.climber_demand;
    }

    public double getClimberPosition() {
        return mPeriodicIO.climber_motor_position;
    }

    public double getClimberCurrent() {
        return mPeriodicIO.climber_stator_current;
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

    public void toggleReleaseSolenoid() {
        mPeriodicIO.release_solenoid = !mPeriodicIO.release_solenoid;
    }

    public void toggleChopsticksSolenoid() {
        mPeriodicIO.chopsticks_solenoid = !mPeriodicIO.chopsticks_solenoid;
    }

    public void toggleHookSolenoid() {
        mPeriodicIO.hook_solenoid = !mPeriodicIO.hook_solenoid;
    }

    public boolean hasEmergency = false;

    public void outputTelemetry() {
        SmartDashboard.putBoolean("Release Solenoid", mPeriodicIO.release_solenoid);
        SmartDashboard.putBoolean("Chopsticks Solenoid", mPeriodicIO.chopsticks_solenoid);
        SmartDashboard.putBoolean("Hook Solenoid", mPeriodicIO.hook_solenoid);
    }

    public static class PeriodicIO {
        /* Inputs */
        public double climber_stator_current;
        public double climber_motor_position;
        public double climber_motor_velocity;

        /* Outputs */
        public double climber_demand;

        public boolean release_solenoid;
        public boolean chopsticks_solenoid;
        public boolean hook_solenoid;
    }
}   
