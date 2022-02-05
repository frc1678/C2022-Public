package com.team1678.frc2022.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;

import edu.wpi.first.wpilibj.Timer;

public class LEDs extends Subsystem {

    public CANdle mCandle = new CANdle(Ports.CANDLE);
    
    public State mState = State.DISABLED;
    public static LEDs mInstance;
    public double mLastTimestamp = Timer.getFPGATimestamp();

    public boolean mRising = false;
    public double mPhase = 0;

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    public enum State {
        OFF(0, 0, 0, Double.POSITIVE_INFINITY, 0.0, false, false),
        DISABLED(255, 20, 30, 4.0, 4.0, false, true), // breathing pink
        ENABLED(0, 0, 255, Double.POSITIVE_INFINITY, 0.0, false, false), // solid blue
        EMERGENCY(255, 0, 0, 0.2, 0.2, false, false), // blinking red
        ONE_BALL(120, 0, 255, 0.7, 0.7, false, false), // blinking green
        TWO_BALL(255, 0, 255, 0.4, 0.4, false, false), // rapid blinking green
        TARGET_VISIBLE(255, 255, 0, Double.POSITIVE_INFINITY, 0.0, false, false), // solid yellow
        SHOT_READY(255, 255, 0, 0.5, 0.5, false, false), // blinking yellow
        RAINBOW(0, 0, 0, 0, 0, true, false); // :)

        int red, green, blue;
        double onTime, offTime;
        boolean breathing;
        boolean isCycleColors;

        /**
         * Creates a new state
         * @param r the red value
         * @param g the green value
         * @param b the blue value
         * @param onTime how long it should take for the color to turn on during the rising edge set to Double.POSITIVE_INFINITY to indicate no animation
         * @param offTime how long it should take for the color to turn off during the falling edge
         * @param isCycleColors overrides all other values and makes the state R A I N B O W
         * @param breathing smooths out the on and off phases with breathing
         */
        private State(int r, int g, int b, double onTime, double offTime, boolean isCycleColors, boolean breathing){
            red = r;
            green = g;
            blue = b;
            this.onTime = onTime;
            this.offTime = offTime;
            this.breathing = breathing;
        }
    }

    public LEDs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        mCandle.configAllSettings(config);

    }

    public void writePeriodicOutputs() {

        if (mState.onTime == Double.POSITIVE_INFINITY) {
            // Positive indicates no animaiton
            mCandle.setLEDs(mState.red, mState.green, mState.blue);
            return;
        }

        double timestamp = (double)System.currentTimeMillis() / 1000d;
        double lastTimestamp = mLastTimestamp;

        // The reason we need a delta time is ignore performance drops without the need for a second thread
        double deltaTime = timestamp - lastTimestamp;
        mLastTimestamp = timestamp;

        // Basic oscillation between 0 and onTime then 0 ofTime
        if (mRising) {
            mPhase = mPhase + deltaTime;
            if (mPhase > mState.onTime) {
                mRising = false;
                mPhase = 0;
            }
        } else {
            mPhase = mPhase + deltaTime;
            if (mPhase > mState.offTime) {
                mRising = true;
                mPhase = 0;
            }
        }

        // Calculate the proper brightness values
        double percentBrightness = 0;
        if (mState.breathing) {
            if (mRising) {
                percentBrightness = (mPhase / mState.onTime);
            } else {
                percentBrightness = 1 - (mPhase / mState.offTime);
            }
        } else {
            percentBrightness = mRising ? 1 : 0;
        }

        int red = mState.red;
        int green = mState.green;
        int blue = mState.blue;

        // Calculate R A I N B O W
        if (mState.isCycleColors) {
            red   = (int)Math.sin(2*timestamp + 2) * 127 + 128;
            green = (int)Math.sin(2*timestamp + 0) * 127 + 128;
            blue  = (int)Math.sin(2*timestamp + 4) * 127 + 128;
        }

        mCandle.setLEDs(red, green, blue);
        mCandle.configBrightnessScalar(percentBrightness);

        


    }

    public State getState() {
        return mState;
    }

    public void setState(State state) {
        mState = state;
    }

    public void updateCANdl() {
        mCandle.setLEDs(mState.red, mState.green, mState.blue);
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

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setState(State.ENABLED);
            }

            @Override
            public void onLoop(double timestamp) {
                updateCANdl();
            }

            @Override
            public void onStop(double timestamp) {
                setState(State.DISABLED);
            }
        });
    }

}