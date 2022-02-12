package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Infrastructure extends Subsystem {

    private static Infrastructure mInstance = new Infrastructure();
    private Compressor mCompressor;

    private boolean mIsDuringAuto = false;

    private Infrastructure() {
        mCompressor = new Compressor(Ports.PCM, PneumaticsModuleType.CTREPCM);
        mCompressor.enableDigital();

    }

    public static Infrastructure getInstance() {
        return mInstance;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void stop() {
        // No-op.
    }

    private void startCompressor() {
        mCompressor.enableDigital();;
    }

    private void stopCompressor() {
        mCompressor.disable();;
    }

    public synchronized void setIsDuringAuto(boolean isDuringAuto) {
        mIsDuringAuto = isDuringAuto;
        if (isDuringAuto)
            stopCompressor();
    }

    public synchronized boolean isDuringAuto() {
        return mIsDuringAuto;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Infrastructure.this) {
                    startCompressor();
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }
}