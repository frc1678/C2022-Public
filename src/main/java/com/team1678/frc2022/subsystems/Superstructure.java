package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.Loop;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.RobotState;
import com.team1678.frc2022.loops.ILooper;

public class Superstructure extends Subsystem {

    /* Superstructure Instance */
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    /* Required Instances */
    // private final Hood mHood = Hood.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    /* Status Variables */
    public double mShooterSetpoint = 0.0;
    public boolean mWantsSpinUp = false;
    public boolean mWantsShoot = false;

    // Aiming Parameters
    public Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    public int mTrackId = -1;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                updateAimingParameters();
                setSetpoints();
                outputTelemetry();
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    public boolean isSpunUp() {
        return mShooter.spunUp();
    }

    public void setWantSpinUp() {
        mWantsSpinUp = !mWantsSpinUp;
    }

    public void setWantSpinUp(boolean spin_up) {
        mWantsSpinUp = spin_up;
    }

    public void setWantShoot(boolean shoot) {
        mWantsShoot = shoot;
    }

    public void setWantShoot() {
        mWantsShoot = !mWantsShoot;
    }

    public void setShooterVelocity(double velocity) {
        mShooterSetpoint = velocity;
    }
    
    public Optional<AimingParameters> getLatestAimingParameters() {
        return mLatestAimingParameters;
    }

    public void updateAimingParameters() {
        mLatestAimingParameters = mRobotState.getAimingParameters(mTrackId, Constants.VisionConstants.kMaxGoalTrackAge);
        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();
        }
    }

    public void setSetpoints() {
        /* Default indexer wanted action to be set */
        Indexer.WantedAction real_indexer;

        if (mWantsSpinUp) {
            mShooter.setVelocity(mShooterSetpoint);
        } else {
            mShooter.setOpenLoop(0.0);
        }

        if (mWantsShoot) {
            if (isSpunUp()) {
                real_indexer = Indexer.WantedAction.FEED;
            } else {
                real_indexer = Indexer.WantedAction.NONE;
            }
        } else {
            /* SET INDEXER STATE */
            if (mIntake.getState() == Intake.State.INTAKING) {
                real_indexer = Indexer.WantedAction.INDEX;
            } else if (mIntake.getState() == Intake.State.REVERSING) {
                real_indexer = Indexer.WantedAction.REVERSE;
            } else {
                real_indexer = Indexer.WantedAction.NONE;
            }
        }

        mIndexer.setState(real_indexer);
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    public void outputTelemetry() {
        SmartDashboard.putBoolean("Want Spin Up", mWantsSpinUp);
        SmartDashboard.putBoolean("Want Shoot", mWantsShoot);
        SmartDashboard.putBoolean("Is Spun Up", isSpunUp());

        /*
        SmartDashboard.putBoolean("Has Aiming Parameters", mLatestAimingParameters.isPresent());
        if (mLatestAimingParameters.isPresent()) {
            SmartDashboard.putNumber("Vehicle to Target", mLatestAimingParameters.get().getRange());
            SmartDashboard.putNumber("Vehicle to TargetAngle", mLatestAimingParameters.get().getVehicleToGoalRotation().getDegrees());

        */
    }

    /*
     * // Subsystem Instance
     * private final Climber mClimber = Climber.getInstance();
     * 
     */
}
