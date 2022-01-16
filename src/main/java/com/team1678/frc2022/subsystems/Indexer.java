package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    private final TalonFX mElevator;
    private final TalonFX mHopperMaster;
    private final TalonFX mHopperSlave;

    private static Indexer mInstance;
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final DigitalInput mBottomBeamBreak = new DigitalInput(Ports.BOTTOM_BEAM_BREAK);
    private final DigitalInput mTopBeamBreak = new DigitalInput(Ports.TOP_BEAM_BREAK);
    //TODO: private final DigitalInput mColorSensor = new DigitalInput(Ports.COLOR_SENOR);

    private State mState = State.IDLE;

    public enum WantedAction {
        NONE,
        INDEX,
        ELEVATE,
        REVERSE,
        HOP,
    }

    public enum State {
        IDLE,
        INDEXING,
        ELEVATING,
        REVERSING,
        HOPPING,
    }

    private Indexer() {
        mElevator = TalonFXFactory.createDefaultTalon(Ports.ELEVATOR_ID);
        mHopperMaster = TalonFXFactory.createDefaultTalon(Ports.HOPPER_MASTER_ID);
        mHopperSlave = TalonFXFactory.createPermanentSlaveTalon(Ports.HOPPER_SLAVE_ID, Ports.HOPPER_MASTER_ID);
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
            case ELEVATE:
                this.mState = State.ELEVATING;
                break;
            case INDEX:
                this.mState = State.INDEXING;
                break;
            case HOP:
                this.mState = State.HOPPING;
                break;
            case NONE:
                this.mState = State.IDLE;
                break;
        }

    }

    public synchronized State getState() {
        return mState;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.topLightBeamBreakSensor = mBottomBeamBreak.get();
        mPeriodicIO.bottomLightBeamBreakSensor = mTopBeamBreak.get();

        mPeriodicIO.hopper_current = mHopperMaster.getStatorCurrent();
        mPeriodicIO.hopper_voltage = mHopperMaster.getMotorOutputVoltage();
    }

    @Override
    public void writePeriodicOutputs() {
        mElevator.set(ControlMode.PercentOutput, mPeriodicIO.elevator_demand / 12.0);
        mHopperMaster.set(ControlMode.PercentOutput, mPeriodicIO.hopper_demand / 12.0);
    }

    @Override
    public void registerEnabledLoops (ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Indexer.this){
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stop();
            }
        });
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

    public double getElevatorDemand() {
        return mPeriodicIO.elevator_demand;
    }

    public double getElevatorCurrent() {
        return mPeriodicIO.elevator_current;
    }
    
    public double getElevatorVoltage() {
        return mPeriodicIO.elevator_voltage;
    }

    public double getHopperDemand() {
        return mPeriodicIO.hopper_demand;
    }

    public double getHopperCurrent() {
        return mPeriodicIO.hopper_current;
    }
    
    public double getHopperVoltage() {
        return mPeriodicIO.hopper_voltage;
    }


    private void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.elevator_demand = 0;
                break;
            case ELEVATING:
                mPeriodicIO.hopper_demand = Constants.IndexerConstants.kIdleVoltage;
                mPeriodicIO.elevator_demand = Constants.IndexerConstants.kElevatorIndexingVoltage;
                break;
            case INDEXING:
                mPeriodicIO.hopper_demand = Constants.IndexerConstants.kHopperIndexingVoltage;
                mPeriodicIO.elevator_demand = Constants.IndexerConstants.kElevatorIndexingVoltage;
                break;
            case HOPPING:
                mPeriodicIO.hopper_demand = Constants.IndexerConstants.kHopperIndexingVoltage;
                mPeriodicIO.elevator_demand = Constants.IndexerConstants.kIdleVoltage;
                break;
            case REVERSING:
                mPeriodicIO.elevator_demand = Constants.IndexerConstants.kElevatorReversingVoltage;
                mPeriodicIO.hopper_demand = Constants.IndexerConstants.kHopperReversingVoltage;
                break;

        }
    }
    
    @Override
    public void stop() {
        mHopperMaster.set(ControlMode.PercentOutput, 0);
        mElevator.set(ControlMode.PercentOutput, 0);

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
        public boolean topLightBeamBreakSensor;
        public boolean bottomLightBeamBreakSensor;
        public double hopper_voltage;
        public double hopper_current;

        //OUTPUTS
        public double elevator_demand;
        public double hopper_demand;
    }


}