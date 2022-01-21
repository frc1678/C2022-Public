package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.subsystems.Intake.State;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper.ColorSensorData;
import com.team254.lib.drivers.TalonFXFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;


public class Indexer extends Subsystem {
    
    private final TalonFX mElevator;
    private final TalonFX mHopperMaster;
    private final TalonFX mHopperSlave;

    private static Indexer mInstance;
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final DigitalInput mBottomBeamBreak;
    private final DigitalInput mTopBeamBreak;
    public REVColorSensorV3Wrapper mColorSensor;

    private final ColorMatch mColorMatcher = new ColorMatch();
    public ColorMatchResult mMatch;

    private final Color mAllianceColor;
    private final Color mOpponentColor;

    private State mState = State.IDLE;

    public enum WantedAction {
        NONE,
        INDEX,
        FEED,
        REVERSE
    }

    public enum State {
        IDLE,
        INDEXING,
        FEEDING,
        REVERSING,
    }

    private Indexer() {
        //mSuperstructure = Superstructure.getInstance();

        mElevator = TalonFXFactory.createDefaultTalon(Ports.ELEVATOR_ID);
        mHopperMaster = TalonFXFactory.createDefaultTalon(Ports.HOPPER_MASTER_ID);

        if (Constants.isAlpha) {
            mHopperSlave = TalonFXFactory.createPermanentSlaveTalon(Ports.HOPPER_SLAVE_ID, Ports.HOPPER_MASTER_ID);
        } else {
            mHopperSlave = null;
        }
      
        mBottomBeamBreak = new DigitalInput(Ports.BOTTOM_BEAM_BREAK);
        mTopBeamBreak = new DigitalInput(Ports.TOP_BEAM_BREAK);
        mColorSensor = new REVColorSensorV3Wrapper(I2C.Port.kOnboard); //TODO: check value

        mHopperMaster.setInverted(true);
        if (Constants.isAlpha) {
            mHopperSlave.setInverted(true);
        }
        
        mHopperSlave.setInverted(true);

        mColorMatcher.addColorMatch(Constants.IndexerConstants.kBlueBallColor);
        mColorMatcher.addColorMatch(Constants.IndexerConstants.kRedBallColor);
        mColorMatcher.addColorMatch(Constants.IndexerConstants.kNeutralColor);

        if (Constants.IndexerConstants.isRedAlliance) {
            mAllianceColor = Constants.IndexerConstants.kRedBallColor;
            mOpponentColor = Constants.IndexerConstants.kBlueBallColor;
        } else {
            mAllianceColor = Constants.IndexerConstants.kBlueBallColor;
            mOpponentColor = Constants.IndexerConstants.kRedBallColor;
        }

        mColorSensor.start();
    }

    public static synchronized Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public synchronized State getState() {
        return mState;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.top_break = !mBottomBeamBreak.get();
        mPeriodicIO.bottom_break = !mTopBeamBreak.get();

        mPeriodicIO.hopper_current = mHopperMaster.getStatorCurrent();
        mPeriodicIO.hopper_voltage = mHopperMaster.getMotorOutputVoltage();

        ColorSensorData reading = mColorSensor.getLatestReading();

        if (reading.color != null) {
            mPeriodicIO.detected_color = reading.color;
            mMatch = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);
        }

        if (mMatch.color == Constants.IndexerConstants.kNeutralColor) {
            mPeriodicIO.eject = false;
        } else if (mMatch.color == mAllianceColor) {
            mPeriodicIO.eject = false;
        } else if (mMatch.color == mOpponentColor) {
            mPeriodicIO.eject = true;
        }

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
    public boolean getTopBeamBreak() {
        return mPeriodicIO.top_break;
    }

    /**
     * Gets the current status of the top beam break
     * @return the state of the beam break
     */
    public boolean getBottomBeamBreak() {
        return mPeriodicIO.bottom_break;
    }

    public String getDetectedColor() {
        return mPeriodicIO.detected_color.toString();
    }

    public String getMatchedColor() {
        return mMatch.toString();
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

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INDEX:
                mState = State.INDEXING;
                break;
            case FEED:
                mState = State.FEEDING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
        }
    }

    private boolean firstBallQueued() {
        return mPeriodicIO.top_break;
    }

    private boolean ballAtElevator() {
        return mPeriodicIO.bottom_break;
    }

    private boolean stopHopper() {
        return ballAtElevator() && firstBallQueued();
    }

    private boolean runElevator() {
        return !firstBallQueued();
    }

    private void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.elevator_demand = Constants.IndexerConstants.kIdleVoltage;
                mPeriodicIO.hopper_demand = Constants.IndexerConstants.kIdleVoltage;
                break;
            case INDEXING:
                mPeriodicIO.hopper_demand = !stopHopper() ? Constants.IndexerConstants.kHopperIndexingVoltage : Constants.IndexerConstants.kIdleVoltage;
                mPeriodicIO.elevator_demand = runElevator() ? Constants.IndexerConstants.kElevatorIndexingVoltage : Constants.IndexerConstants.kIdleVoltage;
                break;
            case FEEDING:
                mPeriodicIO.hopper_demand = Constants.IndexerConstants.kFeedingVoltage;
                mPeriodicIO.elevator_demand = Constants.IndexerConstants.kFeedingVoltage;
                break;
            case REVERSING:
                mPeriodicIO.hopper_demand = Constants.IndexerConstants.kHopperReversingVoltage;
                mPeriodicIO.elevator_demand = Constants.IndexerConstants.kElevatorReversingVoltage;
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
        public Color detected_color;

        public boolean top_break;
        public boolean bottom_break;
        public boolean correctColor;

        //OUTPUTS
        public double elevator_demand;
        public double hopper_demand;
        public boolean eject;
    }


}
