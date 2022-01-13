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

public class Elevator extends Subsystem{

    private final TalonFX mIndexer;

    private static Elevator mInstance;
    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    private boolean mSlotsClean;
    private boolean mHasBall;

    private int mBallCount = 0;

    private final DigitalInput mBottomBeamBreak = new DigitalInput(Constants.ElevatorConstants.kBottomBeamBreak);
    private final DigitalInput mTopBeamBreak = new DigitalInput(Constants.ElevatorConstants.kTopBeamBreak);

    private State mState = State.IDLE;
    private WantedAction mWantedAction = WantedAction.NONE;

    private void spinMotor (double voltage) {
        mPeriodicIO.indexer_demand = voltage;
    }

    private void updateSlots() {
        mHasBall = mPeriodicIO.lightBeamBreakSensor[0];
        mSlotsClean = mPeriodicIO.lightBeamBreakSensor[1];
        if (mBallCount != 0) {
            mSlotsClean = false;
        }
    }

    public enum WantedAction {
        NONE,
        INDEX,
        SHOOT,
        REVERSE,
        /*OUTTAKE_TOP,
        OUTTAKE_BOTTOM*/
    }

    public enum State {
        IDLE, 
        INDEXING, 
        SHOOTING,
        REVERSING,
        /*OUTTAKING_TOP,
        OUTTAKING_BOTTOM*/
    }

    private Elevator() {
        mIndexer = TalonFXFactory.createDefaultTalon(Ports.ELEVATOR_ID);

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

    public static synchronized Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    public void setState (State state) {
        this.mState = state;
    }

    public WantedAction getWantedAction() {
        return mWantedAction;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.lightBeamBreakSensor[0] = mBottomBeamBreak.get();
        mPeriodicIO.lightBeamBreakSensor[1] = mTopBeamBreak.get();
    }

    @Override 
    public void writePeriodicOutputs() {
        mIndexer.set(ControlMode.PercentOutput, mPeriodicIO.indexer_demand / 12.0);
    }

    @Override
    public void registerEnabledLoops (ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.INDEXING;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Elevator.this){
                    runStateMachine();
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

    public void setState (WantedAction wanted_state) {
        mWantedAction = wanted_state;
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INDEX:
                mState = State.INDEXING;
                break;
            case SHOOT:
                mState = State.SHOOTING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
            /*case OUTTAKE_TOP:
                mState = State.OUTTAKING_TOP;
                break;
            case OUTTAKE_BOTTOM:
                mState = State.OUTTAKING_BOTTOM;
                break;*/
        }
    }

    private void runStateMachine() {
        switch (mState) {
            case IDLE:
                spinMotor(0);
                break;
            case INDEXING:
                updateSlots();
                if (mBallCount == 2) {
                    System.out.println("Elevator is full!!");
                } else if (mBallCount < 2 && mBallCount > 0) {
                    spinMotor(Constants.ElevatorConstants.kIndexingVoltage);
                }
                break;
            case SHOOTING:
                spinMotor(Constants.ElevatorConstants.kShootingVoltage);
                break;
            case REVERSING:
                spinMotor(Constants.ElevatorConstants.kReversingVoltage);
                break;
            //Logic for beam breaks, will probably be moved to superstructure
            /*case OUTTAKING_TOP:
                mTopBeamBreak.get();
                if(false){
                    spinMotor(Constants.ElevatorConstants.kIndexingVoltage);
                    mPeriodicIO.indexer_demand = Constants.ElevatorConstants.kIndexingVoltage;
                }if (true) {
                    spinMotor(Constants.ElevatorConstants.kReversingVoltage);
                    mPeriodicIO.indexer_demand = Constants.ElevatorConstants.kReversingVoltage; 
                    mBallCount = mBallCount - 1;
                    mBottomBeamBreak.get();
                    if (true) {
                        setState(State.INDEXING);
                }
                setState(State.INDEXING);
                break;
            }
            case OUTTAKING_BOTTOM:
                spinMotor(Constants.ElevatorConstants.kReversingVoltage);
                mPeriodicIO.indexer_demand = Constants.ElevatorConstants.kReversingVoltage;
                mBallCount = mBallCount - 1;
                break;*/

        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Elevator State", mState.toString());
        SmartDashboard.putNumber("Indexer Demand", mPeriodicIO.indexer_demand);

        SmartDashboard.putNumber("Elevator Voltage", mPeriodicIO.elevator_voltage);
        SmartDashboard.putNumber("Elevator Current", mPeriodicIO.elevator_current);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double elevator_voltage;
        public double elevator_current;
        private boolean[] lightBeamBreakSensor;

        //OUTPUTS
        public double indexer_demand;
    }
    
    
}
