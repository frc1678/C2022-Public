package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends Subsystem {

    private TimeDelayedBoolean mIntakeSolenoidTimer = new TimeDelayedBoolean();

    public enum WantedAction {
        NONE, INTAKE, REVERSE, SPIT, STAY_OUT
    }

    public enum State {
        IDLE, INTAKING, REVERSING, SPITTING, STAYING_OUT
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private static Intake mInstance;
    public State mState = State.IDLE;

    private final TalonFX mMaster;
    private final TalonFX mSingulator;
    private Solenoid mSolenoid;

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ID);
        mSingulator = TalonFXFactory.createPermanentSlaveTalon(Ports.SINGULATOR, Ports.INTAKE_ID);
        mSolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.DEPLOY_SOLENOID_ID);
    }

    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public void setState (State state) {
        this.mState = state;
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    public void zeroSensors() {
    }

    public void runStateMachine() {
        switch (mState) {
            case IDLE:
                mPeriodicIO.demand = 0;
                mPeriodicIO.deploy = false;
                break;
            case INTAKING:
                mPeriodicIO.demand = Constants.IntakeConstants.kIntakingVoltage;
                /*if (mPeriodicIO.intake_out) {
                    mPeriodicIO.demand = Constants.IntakeConstants.kIntakeVoltage;
                } else {
                    mPeriodicIO.demand = Constants.IntakeConstants.kIdleVoltage;
                }*/
                mPeriodicIO.deploy = true;
                break;
            // The if/else statement has been left out, since we might need to reverse while the intake is up
            case REVERSING:
                mPeriodicIO.demand = -Constants.IntakeConstants.kIntakingVoltage;
                mPeriodicIO.deploy = true;
            case SPITTING:
                mPeriodicIO.demand = Constants.IntakeConstants.kSpittingVoltage;
                mPeriodicIO.deploy = false;
                break;
            case STAYING_OUT:
                mPeriodicIO.demand = 0;
                mPeriodicIO.deploy = true;
                break;
        }
    }

    public synchronized State getState() {
        return mState;
    }

    //sets states
   public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INTAKE:
                mState = State.INTAKING;
                break;
            case REVERSE:
                mState = State.REVERSING;
                break;
            case SPIT:
                mState = State.SPITTING;
                break;
            case STAY_OUT:
                mState = State.STAYING_OUT;
                break;
        }
   }

   @Override
   public synchronized void readPeriodicInputs() {
        mPeriodicIO.intake_out = mIntakeSolenoidTimer.update(mPeriodicIO.deploy, 0.2);
        mPeriodicIO.current =  mMaster.getStatorCurrent();
        mPeriodicIO.voltage = mMaster.getMotorOutputVoltage();
       if (mCSVWriter != null) {
           mCSVWriter.add(mPeriodicIO);
       }
   }

   @Override
   public void writePeriodicOutputs() {
       mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
       mSolenoid.set(mPeriodicIO.deploy);
   }

   @Override
   public void registerEnabledLoops(ILooper enabledLooper) {
       enabledLooper.register(new Loop() {
           @Override
           public void onStart(double timestamp) {
               mState = State.IDLE;
           }

           @Override
           public void onLoop(double timestamp) {
               runStateMachine();
           }

           @Override
           public void onStop(double timestamp) {

           }
       });
   }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public static class PeriodicIO {
            // INPUTS
            public double current;
            public boolean intake_out;
            public double voltage;
            
            // OUTPUTS
            public double demand;
            public boolean deploy;
        }

    public double getMotorVoltage() {
        return mPeriodicIO.voltage;
    }

    public boolean getDeployed() {
        return mPeriodicIO.deploy;
    }

    public double getMotorCurrent() {
        return mPeriodicIO.current;  
    }

    public double getMotorDemand() {
        return mPeriodicIO.demand;
    }
}
