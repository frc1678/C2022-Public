package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.Loop;

import edu.wpi.first.wpilibj.Encoder.IndexingType;

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
    // private final Hood mHood = Hood.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Intake mIntake = Intake.getInstance();

    /* Status Variables */
    private boolean mOuttake = true;
    public boolean mWantsShoot = false;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                updateStates();
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

    public void toggleShoot() {
        mWantsShoot = !mWantsShoot;
    }

    public void updateStates() {
        if (mWantsShoot) {
            mShooter.setVelocity(1000);
        } else {
            mShooter.setOpenLoop(0.0);
        }
        if (mIntake.getState() == Intake.State.INTAKING || (mWantsShoot && isSpunUp())) {
            mIndexer.setState(Indexer.WantedAction.INDEX);
        } else if (mIntake.getState() == Intake.State.REVERSING) {
            mIndexer.setState(Indexer.WantedAction.REVERSE);
        } else {
            mIndexer.setState(Indexer.WantedAction.NONE);
        }
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
