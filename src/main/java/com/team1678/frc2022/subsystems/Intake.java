package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.ILooper;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {

    private static double kIntakingVoltage = 5;
    private static double kIdleVoltage = 0;
    public static double kSpittingVoltage = -7;

    //Lists wanted actions
    public enum WantedAction {
        IDLE, INTAKE, REVERSE, RETRACT, SPIT
    }

    //Lists corresponding states
    public enum State {
        IDLING, INTAKING, REVERSING, RETRACTING, SPITTING_OUT
    }

    private State mState = State.IDLE;
    private static PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    //creates two Talon FX motors and a single Solenoid
    private final TalonFX mMaster;
    private final TalonFX mSlave;
    private Solenoid mSolenoid;

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kMasterIntakeRollerId);
        mDeploySolenoid = Constants.makeSolenoidForId(Constants.kDeploySolenoidId);
        mSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kSlaverIntakeRollerId, Constants.kMasterIntakeRollerId);
        //mSlave.setInverted(true); This line will be uncommented if our motor(s) is inverted on the robot
    }

    //Tells us the State the Intake is currently in
    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public void writeToLog() {
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    public void zeroSensors() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    public boolean checkSystem() {
        return true;
    }

    public boolean hasEmergency = false;

    public void runStateMachine() {
        switch (mState) {
            case IDLING:
            mPeriodicIO.demand = kIdleVoltage;
            mPeriodicIO.deploy = false;
            break;
            case INTAKING:
                if (mPeriodicIO.intake_out) {
                    mPeriodicIO.demand = kIntakingVoltage;
                } else {
                    mPeriodicIO.demand = 0.0;
                }
                mPeriodicIO.deploy = true;
                break;
            //The if/else statemenet has been left out, since we might need to reverse while the intake is up
            case REVERSING:
                mPeriodicIO.demand = -kIntakingVoltage;
                break;
            case RETRACTING:
                mPeriodicIO.demand = kIdleVoltage;
                mPeriodicIO.deploy = false;
                break;
            //As in the REVERSING state, the if/else dialouge is omitted
            case SPITTING_OUT:
                mPeriodicIO.demand = kSpittingVoltage;
                break;
        }
    }

    //Returns some values to display on SmartDash
    public double getVoltage() {
        return mPeriodicIO.demand;
    }

    public synchronized State getState() {
        return mState;
    }

    //sets states
    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case IDLE:
                mState = State.IDLING;
                break;
            case INTAKE:
                mState = State.INTAKING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
            case RETRACT:
                mState = State.RETRACTING;
                break;
            case SPIT:
                mState = State.SPITTING_OUT;
                break;
        }
        }


        @Override
        public synchronized void readPeriodicInputs() {
            mPeriodicIO.intake_out = mIntakeSolenoidTimer.update(mPeriodicIO.deploy, 0.2);
            mCurrent = mPeriodicIO.current;
            if (mCSVWriter != null) {
                mCSVWriter.add(mPeriodicIO);
            }
        }
    
        @Override
        public void writePeriodicOutputs() {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
            mDeploySolenoid.set(mPeriodicIO.deploy);
        }
    
        @Override
        public synchronized void outputTelemetry() {
            SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
            SmartDashboard.putString("Intake State", mState.toString());
            if (mCSVWriter != null) {
                mCSVWriter.write();
            }
        }

        public static class PeriodicIO {
            // INPUTS
            public double timestamp;
            public double current;
            public boolean intake_out;
    
            // OUTPUTS
            public double demand;
            public boolean deploy;
        }