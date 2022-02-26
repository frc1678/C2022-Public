package com.team1678.lib.drivers;

import com.revrobotics.ColorSensorV3;
import com.team254.lib.util.CrashTrackingRunnable;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class REVColorSensorV3Wrapper {
    private static final double kPeriod = 0.02;
    private ColorSensorV3 sensor;

    public class ColorSensorData {
        public Color color = null;
        public int distance = 0;
        public double timestamp = 0.0;
    }

    private ColorSensorData mData = null;
    
    private boolean mRunning;
    private final Notifier mNotifier;

    private final Object mReadLock = new Object();
    private double mTimestamp = 0;
    private double mDT = 0;

    private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
            synchronized (mReadLock) {
                if (mRunning) {
                    double now = Timer.getFPGATimestamp();

                    mData.color = sensor.getColor();
                    mData.distance = sensor.getProximity();
                    mData.timestamp = now;

                    mDT = now - mTimestamp;
                    mTimestamp = now;
                }
            }
        }
    };

    public REVColorSensorV3Wrapper(Port port) {
        sensor = new ColorSensorV3(port);

        mNotifier = new Notifier(runnable_);
        mRunning = false;
        mData = new ColorSensorData();
    }

    public ColorSensorData getLatestReading() {
        synchronized (mReadLock) {
            return mData;
        }
    }

    public synchronized void start() {
        if (!mRunning) {
            System.out.println("Starting read of color sensor");

            synchronized (mReadLock) {
                mTimestamp = Timer.getFPGATimestamp();
                mRunning = true;
            }

            mNotifier.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() {
        if (mRunning) {
            mNotifier.stop();

            synchronized (mReadLock) {
                mRunning = false;
                mTimestamp = Timer.getFPGATimestamp();
                System.out.println("Stopping read of color sensor");
            }
        }
    }
    
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Color sensor read rate (Hz)", 1.0 / mDT);
    }
}