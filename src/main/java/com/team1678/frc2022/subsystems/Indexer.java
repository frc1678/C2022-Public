package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper.ColorSensorData;
import com.team254.lib.drivers.TalonFXFactory;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;


public class Indexer extends Subsystem {
    
private static Indexer mInstance;
public PeriodicIO mPeriodicIO = new PeriodicIO();

private TalonFX mOuttake;
private TalonFX mIndexer;
private TalonFX mTrigger;

private final DigitalInput mBottomBeamBreak;
private final DigitalInput mTopBeamBreak;

public boolean mBottomHadSeenBall = false;
public boolean mTopHadSeenBall = false;

public REVColorSensorV3Wrapper mColorSensor;

private final ColorMatch mColorMatcher = new ColorMatch();
ColorMatchResult mMatch;

private static boolean mCorrectColor;

private boolean mRunTrigger() {
    return !mBallAtTrigger() && mPeriodicIO.ball_count > 0;
}

private boolean mBallAtTrigger() {
    return mPeriodicIO.topLightBeamBreakSensor;
}

private State mState = State.IDLE;

public enum WantedAction {
    NONE, INDEX, OUTTAKE, REVERSE, 
}

public enum State{
    IDLE, INDEXING, OUTTAKING, REVERSING,
}

private Indexer() {
    mOuttake = TalonFXFactory.createDefaultTalon(Ports.OUTTAKE_ID);
    mIndexer = TalonFXFactory.createDefaultTalon(Ports.INDEXER_ID);
    mTrigger = TalonFXFactory.createDefaultTalon(Ports.TRIGGER_ID);
    mBottomBeamBreak = new DigitalInput(Ports.BOTTOM_BEAM_BREAK);
    mTopBeamBreak = new DigitalInput(Ports.TOP_BEAM_BREAK);
}
        
    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INDEX:
                mState = State.INDEXING;
                break;
            case OUTTAKE:
                mState = State.OUTTAKING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
        }
    
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.outtake_demand = Constants.IndexerConstants.kOuttakeIdleVoltage;
                mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerIdleVoltage;
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerIdleVoltage;
                break;
            case INDEXING:
                if (mPeriodicIO.correct_Color) {
                    if (mRunTrigger()) {
                        mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerIndexingVoltage;
                    } else {
                        mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerIdleVoltage;
                    }
                        mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerIndexingVoltage;
                    
                    if (mPeriodicIO.bottomLightBeamBreakSensor) {
                        if (!mPeriodicIO.topLightBeamBreakSensor){
                            mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerIndexingVoltage;
                        } else {
                            mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerIdleVoltage;
                        }
                    }
                } else {
                    this.setState(WantedAction.OUTTAKE);
                }
                
                break;
            case OUTTAKING:
                if (mPeriodicIO.correct_Color) {
                    mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerIndexingVoltage;
                    mPeriodicIO.outtake_demand = Constants.IndexerConstants.kOuttakeReversingVoltage;
                } else if (!mPeriodicIO.correct_Color) {
                    mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerReversingVoltage;
                    mPeriodicIO.outtake_demand = Constants.IndexerConstants.kOuttakeIndexingVoltage;
                } else {
                    mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerIdleVoltage;
                    mPeriodicIO.outtake_demand = Constants.IndexerConstants.kOuttakeIdleVoltage;
                }
                break;
            case REVERSING:
                mPeriodicIO.outtake_demand = Constants.IndexerConstants.kOuttakeIndexingVoltage;
                mPeriodicIO.indexer_demand = Constants.IndexerConstants.kIndexerReversingVoltage;
                mPeriodicIO.trigger_demand = Constants.IndexerConstants.kTriggerReversingVoltage;
                break;
        }
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
   

    @Override
    public synchronized void writePeriodicOutputs() {
        mOuttake.set(ControlMode.PercentOutput, mPeriodicIO.outtake_demand/12.0);
        mIndexer.set(ControlMode.PercentOutput, mPeriodicIO.indexer_demand/12.0);  
        mTrigger.set(ControlMode.PercentOutput, mPeriodicIO.trigger_demand/12.0); 
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.topLightBeamBreakSensor = mBottomBeamBreak.get();
        mPeriodicIO.bottomLightBeamBreakSensor = mTopBeamBreak.get();

        if (mPeriodicIO.bottomLightBeamBreakSensor) {
            if (!mBottomHadSeenBall) {
                mBottomHadSeenBall = true;
            }
        } else {
            if (mBottomHadSeenBall) {
                mPeriodicIO.ball_count++;
                mBottomHadSeenBall = false;
            }
        }

        if (mPeriodicIO.topLightBeamBreakSensor) {
            if (!mTopHadSeenBall) {
                mTopHadSeenBall = true;
            }
        } else {
            if (mTopHadSeenBall) {
                mPeriodicIO.ball_count--;
                mTopHadSeenBall = false;
            }
        }
    } 

    public Object getState() {
        return null;
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

    //subsystem setters
    public void setTriggerDemand(double demand) {
        mPeriodicIO.trigger_demand = demand;
    }
    
    public void setOuttakeDemand(double demand) {
        mPeriodicIO.outtake_demand = demand;
    }

    public void setIndexerDemand(double demand) {
        mPeriodicIO.indexer_demand = demand;
    }

    //subsystem getters
    public static Indexer getInstance() {
        return null;
    }

    public double getOuttakeDemand() {
       return mPeriodicIO.outtake_demand;
    }

    public double getOuttakeCurrent() {
        return mPeriodicIO.outtake_current;
    }

    public double getOuttakeVoltage() {
        return mPeriodicIO.outtake_voltage;
    }

    public double getIndexerCurrent() {
        return mPeriodicIO.indexer_current;
    }

    public double getIndexerDemand() {
        return mPeriodicIO.indexer_demand;
    }

    public double getIndexerVoltage() {
        return mPeriodicIO.indexer_voltage;
    }

    public double getTriggerCurrent() {
        return mPeriodicIO.trigger_current;
    }

    public double getTriggerDemand() {
        return mPeriodicIO.trigger_demand;
    }

    public double getTriggerVoltage() {
        return mPeriodicIO.trigger_voltage;
    }

    public double getBallCount() {
        return mPeriodicIO.ball_count;
    }

    public boolean getTopBeamBreak() {
        return mPeriodicIO.topLightBeamBreakSensor;
    }

    public boolean getBottomBeamBreak() {
        return mPeriodicIO.bottomLightBeamBreakSensor;
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean topLightBeamBreakSensor;
        public boolean bottomLightBeamBreakSensor;
        public double ball_count;
        
        public double outtake_current;
        public double indexer_current;
        public double trigger_current;
        
        public double outtake_voltage;
        public double indexer_voltage;
        public double trigger_voltage;

        public boolean correct_Color;
        public Color detected_color;

        // OUTPUTS
        public double outtake_demand;
        public double indexer_demand;
        public double trigger_demand;

        public boolean eject;
    }

    // only call for quick status testing
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Top Had Seen Ball", mTopHadSeenBall);
        SmartDashboard.putBoolean("Bottom Had Seen Ball", mBottomHadSeenBall);
    }
    
}
   

    