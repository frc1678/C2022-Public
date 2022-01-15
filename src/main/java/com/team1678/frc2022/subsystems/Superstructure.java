package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.loops.ILooper;

public class Superstructure extends Subsystem{

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
    private boolean mOuttake = false;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
       enabledLooper.register(new Loop() {
           @Override
           public void onStart(double timestamp) {
               
           }

           @Override
           public void onLoop(double timestamp) {
               updateIndexerStates();
               
           }

           @Override
           public void onStop(double timestamp) {

           }
       });
   }

   public synchronized void updateIndexerStates() {

        final boolean mIndexerBottomSensor = mIndexer.bottomBeamBreak();
        final boolean mIndexerTopSensor = mIndexer.topBeamBreak();
        final boolean mCorrectColor = false /*= mIndexer.getCorrectColor()*/;

        Indexer.WantedAction indexer_state;

        if (mIndexerBottomSensor) {
            if (mCorrectColor) {
                mOuttake = false;
                if (mIndexerTopSensor) {
                    indexer_state = Indexer.WantedAction.NONE;
                } else {
                    indexer_state = Indexer.WantedAction.INDEX;
                }
            } else {
                mOuttake = true;
                indexer_state = Indexer.WantedAction.NONE;
            }
        } else {
            mOuttake = false;
            if (mIndexerTopSensor) {
                indexer_state = Indexer.WantedAction.HOP;
            } else {
                indexer_state = Indexer.WantedAction.INDEX;
            }
        }

        mIndexer.setState(indexer_state);
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

    /*
    // Subsystem Instance
    private final Climber mClimber = Climber.getInstance();

    */
}
