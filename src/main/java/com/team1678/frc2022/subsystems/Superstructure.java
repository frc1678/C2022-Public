package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.subsystems.Intake.WantedAction;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team1678.frc2022.Constants;
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

    /* Required Subsystem Instances */
    private final Hood mHood = Hood.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Intake mIntake = Intake.getInstance();

    /* Status Variables */
    public double mFlywheelSetpoint = 0.0;
    public double mAcceleratorSetpoint = 0.0;
    public double mHoodSetpoint = 20.0;
    public boolean mWantSpinUp = false;
    public boolean mWantShoot = false;
    public boolean mWantIntake = false;
    public boolean mWantOuttake = false;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                setSetpoints();
                SmartDashboard.putBoolean("Want Spin Up", mWantSpinUp);
                SmartDashboard.putBoolean("Want Shoot", mWantShoot);
                SmartDashboard.putBoolean("Is Spun Up", isSpunUp());
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
        mWantSpinUp = !mWantSpinUp;
    }

    public void setWantSpinUp(boolean spin_up) {
        mWantSpinUp = spin_up;
    }

    public void setWantShoot(boolean shoot) {
        mWantShoot = shoot;
    }

    public void setWantShoot() {
        mWantShoot = !mWantShoot;
    }

    public void setShooterVelocity(double flywheelVelocity, double acceleratorVelocity) {
        mFlywheelSetpoint = flywheelVelocity;
        mAcceleratorSetpoint = acceleratorVelocity;
    }

    public void setWantIntake() {
        setWantIntake(!mWantIntake);
    }

    public void setWantIntake(boolean intake) {
        if (mWantIntake != intake) {
            mWantIntake = intake;
        }
    }

    public void setWantOuttake() {
        setWantIntake(!mWantIntake);
    }

    public void setWantOuttake(boolean outtake) {
        if (mWantOuttake != outtake) {
            mWantOuttake = outtake;
        }
    }

    public void setSetpoints() {
        /* Default indexer wanted action to be set */
        Indexer.WantedAction real_indexer = Indexer.WantedAction.NONE;

        if (mWantSpinUp) {
            mShooter.setVelocity(mFlywheelSetpoint, mAcceleratorSetpoint);
        } else {
            mShooter.setOpenLoop(0.0, 0.0);
        }

        double real_hood = Util.clamp(mHoodSetpoint,
                Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit,
                Constants.HoodConstants.kHoodServoConstants.kMaxUnitsLimit);

        if (mWantShoot) {
            if (isSpunUp()) {
                real_indexer = Indexer.WantedAction.FEED;
            }
        } else {
            if (mWantIntake) {
                real_indexer = Indexer.WantedAction.INDEX;
                mIntake.setState(Intake.WantedAction.INTAKE);
            } else if (mWantOuttake) {
                mIntake.setState(Intake.WantedAction.REVERSE);
                real_indexer = Indexer.WantedAction.REVERSE;
            } else {
                mIntake.setState(WantedAction.NONE);
            }
        }


        mIndexer.setState(real_indexer);
        mHood.setSetpointMotionMagic(real_hood);
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    /*
     * // Subsystem Instance
     * private final Climber mClimber = Climber.getInstance();
     * 
     */
}
