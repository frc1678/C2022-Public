package com.team1678.frc2022.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;

public class LEDs extends Subsystem {

    public CANdle mCandle = new CANdle(Ports.CANDLE);
    
    public State mState = State.DISABLED;
    public LEDs mInstance;

    public LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    public enum State {
        OFF(0, 0, 0, Double.POSITIVE_INFINITY, 0.0, false),
        DISABLED(255, 20, 30, Double.POSITIVE_INFINITY, 0.0, false), // solid pink
        ENABLED(0, 0, 255, Double.POSITIVE_INFINITY, 0.0, false), // solid blue
        EMERGENCY(255, 0, 0, 0.2, 0.2, false), // blinking red
        ONE_BALL(120, 0, 255, 0.7, 0.7, false), // blinking green
        TWO_BALL(255, 0, 255, 0.4, 0.4, false), // rapid blinking green
        TARGET_FOUND(255, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // solid yellow
        SHOT_READY(255, 255, 0, 0.5, 0.5, false), // blinking yellow
        RAINBOW(0, true); // :)

        int red, green, blue;
        double onTime, offTime, cycleTime, transitionTime;
        float startingHue;
        List<List<Double>> colors = new ArrayList<List<Double>>();
        boolean isCycleColors;
        private State(int r, int g, int b, double onTime, double offTime, boolean isCycleColors){
            red = r;
            green = g;
            blue = b;
            this.onTime = onTime;
            this.offTime = offTime;
        }

        private State(float hue, boolean cycle) {
            this.startingHue = hue;
            this.isCycleColors = cycle;
        }

        private State(float hue, double transTime, boolean cycle) {
            this.startingHue = hue;
            this.transitionTime = transTime;
            this.isCycleColors = cycle;
        }

        private State(List<List<Double>> colors, double cycleTime, boolean isCycleColors, double transitionTime) {
            this.colors = colors;
            this.cycleTime = cycleTime;
            this.isCycleColors = isCycleColors;
            this.transitionTime = transitionTime;
        }
    }

    public LEDs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        mCandle.configAllSettings(config);

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