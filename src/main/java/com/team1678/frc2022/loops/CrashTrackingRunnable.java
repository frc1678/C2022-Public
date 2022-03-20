package com.team1678.frc2022.loops;

// CREDIT - FRC Team 1323

/**
 * Runnable class with reports all uncaught throws to CrashTracker
 */
public abstract class CrashTrackingRunnable implements Runnable {

    @Override
    public final void run() {
        try {
            runCrashTracked();
        } catch (Throwable t) {
            t.printStackTrace();
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public abstract void runCrashTracked();
}
