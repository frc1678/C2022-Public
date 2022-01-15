package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem{

    private final TalonFX mIndexer;
    private final TalonFX mHopper;

    private static Indexer mInstance;
    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    private boolean mSlotsClean;
    private boolean mHasBall;

    private int mBallCount = 0;

    private final DigitalInput mBottomBeamBreak = new DigitalInput(Constants.ElevatorConstants.kBottomBeamBreak);
    private final DigitalInput mTopBeamBreak = new DigitalInput(Constants.ElevatorConstants.kTopBeamBreak);

    private State mState = State.IDLE;

    public enum WantedAction {
        NONE,
        INDEX,
        REVERSE,
        HOP,
    }

    public enum State {
        IDLE,
        ELEVATING,
        REVERSING,
        HOPPING,
    }

    private Indexer() {
        mIndexer = TalonFXFactory.createDefaultTalon(Ports.ELEVATOR_ID);
        mHopper = TalonFXFactory.createDefaultTalon(Ports.HOPPER_ID);

        mIndexer.config_kP(0, Constants.ElevatorConstants.kIndexerKp, Constants.kLongCANTimeoutMs);
        mIndexer.config_kI(0, Constants.ElevatorConstants.kIndexerKi, Constants.kLongCANTimeoutMs);
        mIndexer.config_kD(0, Constants.ElevatorConstants.kIndexerKd, Constants.kLongCANTimeoutMs);
        mIndexer.config_kF(0, Constants.ElevatorConstants.kIndexerKf, Constants.kLongCANTimeoutMs);
        mIndexer.config_kP(1, Constants.ElevatorConstants.kIndexerVelocityKp, Constants.kLongCANTimeoutMs);
        mIndexer.config_kI(1, Constants.ElevatorConstants.kIndexerVelocityKi, Constants.kLongCANTimeoutMs);
        mIndexer.config_kD(1, Constants.ElevatorConstants.kIndexerVelocityKd, Constants.kLongCANTimeoutMs);
        mIndexer.config_kF(1, Constants.ElevatorConstants.kIndexerVelocityKf, Constants.kLongCANTimeoutMs);

        mIndexer.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mIndexer.configMotionCruiseVelocity(Constants.ElevatorConstants.kIndexerMaxVelocity);
        mIndexer.configMotionAcceleration(Constants.ElevatorConstants.kIndexerMaxAcceleration);

        mIndexer.set(ControlMode.PercentOutput, 0);
        mIndexer.setInverted(false);
        mIndexer.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mIndexer.enableVoltageCompensation(true);

        mIndexer.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        mIndexer.configClosedloopRamp(0.0);
    }

    public static synchronized Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public void setState(WantedAction state) {
        switch (state) {
            case REVERSE:
                this.mState = State.REVERSING;
                break;
            case INDEX:
                this.mState = State.ELEVATING;
                break;
            case HOP:
                this.mState = State.HOPPING;
                break;
            case NONE:
                this.mState = State.IDLE;
                break;
        }

    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.topLightBeamBreakSensor = mBottomBeamBreak.get();
        mPeriodicIO.bottomLightBeamBreakSensor = mTopBeamBreak.get();

        mPeriodicIO.hopper_current = mHopper.getStatorCurrent();
        mPeriodicIO.hopper_voltage = mHopper.getMotorOutputVoltage();
    }

    @Override 
    public void writePeriodicOutputs() {
        mIndexer.set(ControlMode.PercentOutput, mPeriodicIO.indexer_demand / 12.0);
        mHopper.set(ControlMode.PercentOutput, mPeriodicIO.hopper_demand / 12.0);
    }

    @Override
    public void registerEnabledLoops (ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.ELEVATING;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Indexer.this){
                    runStateMachine();
                    updateSlots();
                    outputTelemetry();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stop();
            }
        });
    }


    private void updateSlots() {
        // Update beam breaks
        mPeriodicIO.topLightBeamBreakSensor = mTopBeamBreak.get();
        mPeriodicIO.bottomLightBeamBreakSensor = mBottomBeamBreak.get();

        // Update ball count
        mBallCount = 0;
        if (mPeriodicIO.topLightBeamBreakSensor) mBallCount += 1;
        if (mPeriodicIO.bottomLightBeamBreakSensor) mBallCount += 1;

        // Update slot clean status
        mSlotsClean = (mBallCount == 0);
    }

    /**
     * Gets the current status of the top beam break
     * @return the state of the beam break
     */
    public boolean topBeamBreak() {
        return mPeriodicIO.topLightBeamBreakSensor;
    }

    /**
     * Gets the current status of the top beam break
     * @return the state of the beam break
     */
    public boolean bottomBeamBreak() {
        return mPeriodicIO.bottomLightBeamBreakSensor;
    }

    /**
     * Gets the count of the balls (between 0 and 2)
     * @return how many balls are in the system
     */
    public int ballCount() {
        return mBallCount;
    }

    /**
     * Returns if the indexer is clear
     * @return if the indexer is clear
     */
    public boolean slotsClean() {
        return mSlotsClean;
    }

    private void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.indexer_demand = 0;
                break;
            case ELEVATING:
                mPeriodicIO.hopper_demand = Constants.ElevatorConstants.kIdleVoltage;
                mPeriodicIO.indexer_demand = Constants.ElevatorConstants.kIndexingVoltage;
                break;
            case HOPPING:
                mPeriodicIO.hopper_demand = Constants.ElevatorConstants.kHopperIndexingVoltage;
                mPeriodicIO.indexer_demand = Constants.ElevatorConstants.kIdleVoltage;
                break;
            case REVERSING:
                mPeriodicIO.indexer_demand = Constants.ElevatorConstants.kReversingVoltage;
                break;

        }
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Indexer State", mState.toString());
        SmartDashboard.putNumber("Indexer Demand", mPeriodicIO.indexer_demand);

        SmartDashboard.putNumber("Elevator Voltage", mPeriodicIO.elevator_voltage);
        SmartDashboard.putNumber("Elevator Current", mPeriodicIO.elevator_current);

        SmartDashboard.putNumber("Hopper Demand", mPeriodicIO.hopper_demand);
        SmartDashboard.putNumber("Hopper Voltage", mPeriodicIO.hopper_voltage);
        SmartDashboard.putNumber("Hopper Current", mPeriodicIO.hopper_current);
    }

    @Override
    public void stop() {
        mHopper.set(ControlMode.PercentOutput, 0);
        mIndexer.set(ControlMode.PercentOutput, 0);
        
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double elevator_voltage;
        public double elevator_current;
        private boolean topLightBeamBreakSensor;
        private boolean bottomLightBeamBreakSensor;
        public double hopper_voltage;
        public double hopper_current;

        //OUTPUTS
        public double indexer_demand;
        public double hopper_demand;
    }
    
    
}
