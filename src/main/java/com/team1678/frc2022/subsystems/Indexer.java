package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.lib.drivers.BeamBreak;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {
    
    private static Indexer mInstance;
    public static synchronized Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    private TalonFX mEjector;
    private TalonFX mTunnel;

    private final BeamBreak mBottomBeamBreak;
    private final BeamBreak mTopBeamBreak;

    private Timer ejectDelayTimer = new Timer();
    
    public boolean stopTunnel() {
        return ballAtTrigger() && ballInTunnel();
    }

    public boolean ballAtTrigger() {
        return mPeriodicIO.top_break;
    }

    public boolean ballInTunnel () {
        return mPeriodicIO.bottom_break;
    }

    private IndexerSlot mTopSlot = new IndexerSlot();
    private IndexerSlot mBottomSlot = new IndexerSlot();

    private boolean mIndexingTopBall, mIndexingBottomBall, mEjecting, mFeeding = false;


    private Indexer() {
        mEjector = TalonFXFactory.createDefaultTalon(Ports.EJECTOR_ID);
        mTunnel = TalonFXFactory.createDefaultTalon(Ports.TUNNEL_ID);

        /* Tuning Values */
        mTunnel.config_kP(0, Constants.IndexerConstants.kTunnelP, Constants.kLongCANTimeoutMs);
        mTunnel.config_kI(0, Constants.IndexerConstants.kTunnelI, Constants.kLongCANTimeoutMs);
        mTunnel.config_kD(0, Constants.IndexerConstants.kTunnelD, Constants.kLongCANTimeoutMs);
        mTunnel.config_kF(0, Constants.IndexerConstants.kTunnelF, Constants.kLongCANTimeoutMs);
        mTunnel.config_IntegralZone(0, (int) (200.0 / Constants.IndexerConstants.kTunnelVelocityConversion));
        mTunnel.selectProfileSlot(0, 0);
        mTunnel.configClosedloopRamp(0.1);

        mTunnel.setInverted(true);

        mBottomBeamBreak = new BeamBreak(Ports.getBottomBeamBreakPort());
        mTopBeamBreak = new BeamBreak(Ports.getTopBeamBreakPort());
    }
    
    @Override
    public void registerEnabledLoops (ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setWantNone();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Indexer.this){
                    updateSetpoints();
                    // send log data
                    SendLog();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public void updateSetpoints() {

        // Update slots according to beam breaks
        mTopSlot.updateHasBall(getTopBeamBreak());
        mBottomSlot.updateHasBall(getBottomBeamBreak());

        if (mFeeding) { // Feeding to the shooter
            mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kTunnelFeedingVelocity;
            mPeriodicIO.ejector_demand = Constants.IndexerConstants.kEjectorFeedingVoltage;
        } else if (mEjecting) { // Pooping
            mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kTunnelIndexingVelocity;
            mPeriodicIO.ejector_demand = -Constants.IndexerConstants.kEjectorVoltage;
            
            // Keep ejecting for X amount of seconds to ensure ball has left the system
            if (mBottomBeamBreak.getCleared()) {
                ejectDelayTimer.reset();
                ejectDelayTimer.start();
            } 
        if (ejectDelayTimer.hasElapsed(0.5)) {
                System.out.println("stopped ejecting");
                ejectDelayTimer.stop();
                ejectDelayTimer.reset();
                mEjecting = false;
            }
        } else if (mIndexingTopBall) { // Indexing first ball
            mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kTunnelIndexingVelocity;
            mPeriodicIO.ejector_demand = Constants.IndexerConstants.kEjectorVoltage;

            // Stop running indexer when top slot has a ball
            if (mTopSlot.hasBall()) {
                mIndexingTopBall = false;
            }

        } else if (mIndexingBottomBall) { // Indexing second ball
            mPeriodicIO.tunnel_demand = Constants.IndexerConstants.kTunnelIndexingVelocity;
            mPeriodicIO.ejector_demand = Constants.IndexerConstants.kEjectorVoltage;

            // Stop running indexer when bottom slot has a ball
            if (mBottomSlot.hasBall()) {
                mIndexingBottomBall = false;
            }
        } else {
            mPeriodicIO.tunnel_demand = 0.0;
            mPeriodicIO.ejector_demand = 0.0;
        }
    }

    // Stop running indexer
    public void setWantNone() {
        mIndexingTopBall = false;
        mIndexingBottomBall = false;
        mEjecting = false;
    }

    // Reset slots
    public void clearSlots() {
        mTopSlot = new IndexerSlot();
        mBottomSlot = new IndexerSlot();
    }

    // Queue a ball for indexing
    public void queueBall(boolean color) {
        if (!mTopSlot.hasBall() && !mTopSlot.hasQueuedBall()) {
            queueFirstBall(color);
        } else if (!mBottomSlot.hasBall() && !mBottomSlot.hasQueuedBall()) {
            queueSecondBall(color);
        }
    }

    private void queueFirstBall(boolean color) {
        mTopSlot.queueBall(color);
        mIndexingTopBall = true;
    }

    private void queueSecondBall(boolean color) {
        mBottomSlot.queueBall(color);
        mIndexingBottomBall = true;
    }

    // Feed to shooter
    public void setWantFeeding(boolean wantsFeed) {
        mFeeding = wantsFeed;
    }

    // Queue a ball for ejection
    public void queueEject() {
        mEjecting = true;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mEjector.set(ControlMode.PercentOutput, mPeriodicIO.ejector_demand / 12.0);
        if (mPeriodicIO.tunnel_demand == 0.0) {
            mTunnel.set(ControlMode.PercentOutput, 0.0);
        } else {
            mTunnel.set(ControlMode.Velocity,
                    mPeriodicIO.tunnel_demand / Constants.IndexerConstants.kTunnelVelocityConversion);
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mTopSlot.outputToSmartdash("Top");
        mBottomSlot.outputToSmartdash("Bottom");
        SmartDashboard.putBoolean("Indexing Top Ball", mIndexingTopBall);
        SmartDashboard.putBoolean("Indexing Bottom Ball", mIndexingBottomBall);
        SmartDashboard.putBoolean("Indexing Feeding", mFeeding);
        SmartDashboard.putBoolean("Indexing Ejecting", mEjecting);
        mPeriodicIO.top_break = !mTopBeamBreak.get();
        mPeriodicIO.bottom_break = !mBottomBeamBreak.get();

        mPeriodicIO.tunnel_velocity = mTunnel.getSelectedSensorVelocity() * Constants.IndexerConstants.kTunnelVelocityConversion;
        mPeriodicIO.tunnel_current = mTunnel.getStatorCurrent();
        mPeriodicIO.tunnel_voltage = mTunnel.getMotorOutputVoltage();

        mPeriodicIO.ejector_velocity = mEjector.getSelectedSensorVelocity();
        mPeriodicIO.ejector_current = mEjector.getStatorCurrent();
        mPeriodicIO.ejector_voltage = mEjector.getMotorOutputVoltage();
    }

    @Override
    public void stop() {
        setWantNone();
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    } 

    // subsystem setters
    
    public void setOuttakeDemand(double demand) {
        mPeriodicIO.ejector_demand = demand;
    }

    public void setIndexerDemand(double demand) {
        mPeriodicIO.tunnel_demand = demand;
    }

    public double getEjectorDemand() {
       return mPeriodicIO.ejector_demand;
    }

    public double getEjectorCurrent() {
        return mPeriodicIO.ejector_current;
    }

    public double getEjectorVoltage() {
        return mPeriodicIO.ejector_voltage;
    }

    public double getTunnelCurrent() {
        return mPeriodicIO.tunnel_current;
    }

    public double getTunnelDemand() {
        return mPeriodicIO.tunnel_demand;
    }

    public double getTunnelVoltage() {
        return mPeriodicIO.tunnel_voltage;
    }

    public boolean getTopBeamBreak() {
        return mPeriodicIO.top_break;
    }

    public boolean getBottomBeamBreak() {
        return mPeriodicIO.bottom_break;
    }

    public boolean indexerFull() {
        return (mTopSlot.hasBall() || mTopSlot.hasQueuedBall()) && (mBottomSlot.hasBall() || mBottomSlot.hasQueuedBall());
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean top_break;
        public boolean bottom_break;

        public double tunnel_velocity;
        public double ejector_velocity;
        
        public double ejector_current;
        public double tunnel_current;
        
        public double ejector_voltage;
        public double tunnel_voltage;

        // OUTPUTS
        public double ejector_demand;
        public double tunnel_demand;
    }

    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "INDEXER_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");
        headers.add("bottom_break");
        headers.add("top_break");
        headers.add("ejector_current");
        headers.add("ejector_voltage");
        headers.add("tunnel_voltage");
        headers.add("tunnel_demand");
        headers.add("ejector_demand");
        headers.add("tunnel_current");
        headers.add("tunnel_velocity");
        
        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());
        items.add(mPeriodicIO.bottom_break ? 1.0 : 0.0);
        items.add(mPeriodicIO.top_break ? 1.0 : 0.0);
        items.add(mPeriodicIO.ejector_current);
        items.add(mPeriodicIO.ejector_voltage);
        items.add(mPeriodicIO.tunnel_voltage);
        items.add(mPeriodicIO.tunnel_demand);
        items.add(mPeriodicIO.ejector_demand);
        items.add(mPeriodicIO.tunnel_current);
        items.add(mPeriodicIO.tunnel_velocity);

        // send data to logging storage
        mStorage.addData(items);
    }

    private class IndexerSlot {
        private boolean hasBall;
        private boolean correctColor;
        
        private boolean hasQueuedBall;
        private boolean queuedBallColor;

        public IndexerSlot() {
            hasBall = false;
            correctColor = false;
            hasQueuedBall = false;
            queuedBallColor = false;
        }

        public boolean hasBall() {
            return hasBall;
        }

        public boolean hasCorrectColor() {
            return hasBall && correctColor;
        }

        public void updateHasBall(boolean beamBreakRead) {
            hasBall = beamBreakRead;
            if (beamBreakRead) {
                if (hasQueuedBall) {
                    hasQueuedBall = false;
                    correctColor = queuedBallColor;
                }
            } else {
                hasQueuedBall = false;
            }

        }

        public void queueBall(boolean correctColor) {
            hasQueuedBall = true;
            queuedBallColor = correctColor;
        }

        public boolean hasQueuedBall() {
            return hasQueuedBall;
        }

        public void outputToSmartdash(String name) {
            SmartDashboard.putBoolean(name + " has ball", hasBall);
            SmartDashboard.putBoolean(name + " has correct color", correctColor);
            SmartDashboard.putBoolean(name + " has queued ball", hasQueuedBall);
            SmartDashboard.putBoolean(name + " queued ball color", queuedBallColor);
        }
    }
    
}
   

    