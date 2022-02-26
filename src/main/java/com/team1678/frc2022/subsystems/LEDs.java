package com.team1678.frc2022.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class LEDs extends Subsystem {

    public CANdle mCandle = new CANdle(Ports.CANDLE, "canivore1");
    
    public State mBottomState = State.POLICE_BLUE;
    public State mTopState = State.POLICE_RED;

    public StateStatus topStatus = new StateStatus();
    public StateStatus bottomStatus = new StateStatus();

    public LEDSector mLeftTopSector = new LEDSector(8, 6, 1);
    public LEDSector mLeftBottomSector = new LEDSector(14, 7, 1);
    public LEDSector mRightTopSector = new LEDSector(21, 7, 1);
    public LEDSector mRightBottomSector = new LEDSector(28, 6, 1);

    public static LEDs mInstance;
    public double mLastTimestamp = Timer.getFPGATimestamp();

    public int sector = 0;

    public final Superstructure mSuperstructure = Superstructure.getInstance();
    public final Indexer mIndexer = Indexer.getInstance();

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    public enum State {

        // Robot status indicators
        OFF(0, 0, 0, Double.POSITIVE_INFINITY, 0.0, false), // LEDs OFF
        DISABLED(255, 20, 30, 9.0, 4.0, true), // Pink Breathing
        IDLE(0, 0, 255, Double.POSITIVE_INFINITY, 0.0, false), // Blue
        EMERGENCY(255, 0, 0, 0.3, 0.3, false), // Red Blinking

        // Test Colors
        RED(255, 0, 0, Double.POSITIVE_INFINITY, 0.0, false), // RED
        GREEN(0, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // GREEN
        BLUE(0, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // BLUE
        WHITE(255, 255, 255, 10, 5, false), // WHITE BLINKING
        MAGENTA(255, 0, 255, Double.POSITIVE_INFINITY, 0.0, false), // MAGENTA

        // Strip States
        ONE_BALL(0.5, 0.2, false, new Color(0.5, 0.1, 0.1), new Color(0.1, 0.1, 0.5)), // Alternating Yellow Blue
        TWO_BALL(Double.POSITIVE_INFINITY, 0.05, false, new Color(0.5, 0.1, 0.1)), // Fast Alternating Yellow Red

        // Regular States
        TARGET_VISIBLE(255, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // Slow Yellow
        LOCKED_ON(0, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // Solid Gren
        SHOT_READY(0, 255, 0, 0.05, 0.05, false), // FAST Green

        // Super States
        CLIMB_MODE(255, 0, 255, Double.POSITIVE_INFINITY, 0.0, false),

        // Fun Colors!
        MERICA(0.3, 0, false, new Color(1, 1, 1), new Color(1, 0, 0), new Color(0, 0, 1)), // Unused
        CITRUS(0.5, 0.5, true, new Color(0.23, 0.83, 0.18), new Color(0, 1, 0)), // Unused
        WORSE_RAINBOW(5, 0.5, true, new Color(1, 0, 0), new Color(1, 0.5, 0), new Color(1, 1, 0), new Color(0, 1, 0), new Color(0, 0, 1), new Color(0.29, 0, 0.51), new Color(0.58, 0, 0.83)), // Unused
        POLICE_RED(0.15, 0.0, true, new Color(1, 0, 0), new Color(0, 0, 0)),
        POLICE_BLUE(0.15, 0.0, true, new Color(0, 0, 0), new Color(0, 0, 1)),
        DAVID(0.1, 0.1, true, new Color(5, 2, 9), new Color(5, 0, 9)),
        RAINBOW(); // :)

        List<Color> colors = new ArrayList<Color>();
        double onTime, offTime;
        boolean breathing = false;
        boolean isCycleColors = false;

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
        private State(double r, double g, double b, double onTime, double offTime, boolean breathing){
            colors.add(new Color(r/255, g/255, b/255));
            this.onTime = onTime;
            this.offTime = offTime;
            this.breathing = breathing;
        }

        private State() {
            isCycleColors = true;
            this.onTime = 100;
            this.offTime = 0;
            this.breathing = false;
        }

        private State(double onTime, double offTime, boolean breathing, Color... colors) {
            this.onTime = onTime;
            this.offTime = offTime;
            this.breathing = breathing;
            for (Color color : colors) {
                this.colors.add(color);
            }

        }
    }

    private class StateStatus {
        public int colorPhase = 0;
        public double phase = 0;
        public boolean rising = false;

        public StateStatus() {
            colorPhase = 0;
            phase = 0;
            rising = false;
        }
    }

    /**
     * This class holds information about the led sectors and their current status
     */
    private class LEDSector {
        public int startLed = 0;
        public int ledCount = 0;
        public double brightness = 1;

        /**
         * Create a new instance of LEDSector
         * @param startLed the starting ID of the first LED (0-512)
         * @param ledCount
         * @param brightness
         */
        public LEDSector(int startLed, int ledCount, double brightness) {
            this.startLed = startLed;
            this.ledCount = ledCount;
            this.brightness = brightness;
        }
    }

    public LEDs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.disableWhenLOS = true;
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1.0; // global brightness scale
        mCandle.configAllSettings(config, Constants.kLongCANTimeoutMs);
        mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);

    }

    public double getCurrentDraw() {
        return mCandle.getCurrent();
    }

    public double getTemperature() {
        return mCandle.getTemperature();
    }

    public void updateState() {
        if (mSuperstructure.hasEmergency) {
            setBottomState(State.EMERGENCY);
            setTopState(State.EMERGENCY);
            return;
        }

        if (mSuperstructure.getBallCount() == 1) {
            setTopState(State.ONE_BALL);
        } else if (mSuperstructure.getBallCount() == 2) {
            setTopState(State.TWO_BALL);
        } else {
            setTopState(State.IDLE);
        }

        if (Limelight.getInstance().hasTarget()) {
            setBottomState(State.TARGET_VISIBLE);
        } else if (mSuperstructure.isAimed()) {
            setBottomState(State.LOCKED_ON);
        } else if (mSuperstructure.isSpunUp()) {
            setBottomState(State.SHOT_READY);
        } else {
            setBottomState(State.IDLE);
        }

        if (mSuperstructure.getInClimbMode()) {
            setTopState(State.CLIMB_MODE);
            setBottomState(State.CLIMB_MODE);
        }
    }

    public void updateLights() {

        double timestamp = Timer.getFPGATimestamp();
        double lastTimestamp = mLastTimestamp;

        // The reason we need a delta time is ignore performance drops without the need for a second thread
        double deltaTime = timestamp - lastTimestamp;
        mLastTimestamp = timestamp;

        // For some reason we need to alternate setting states
        
        switch (sector) {
            case 0:
                updateLEDsFromState(mBottomState, this.mLeftBottomSector, bottomStatus, deltaTime * 2, timestamp);
                break;
            case 1:
                updateLEDsFromState(mTopState, this.mRightBottomSector, topStatus, deltaTime * 2, timestamp);
                break;
            case 2:
                updateLEDsFromState(mTopState, this.mLeftTopSector, topStatus, deltaTime * 2, timestamp);
                break;
            case 3:
                updateLEDsFromState(mBottomState, this.mRightTopSector, bottomStatus, deltaTime * 2, timestamp);
                break;
        }
        SmartDashboard.putNumber("LED Deltatime", deltaTime);
        
        sector++;
        if (sector > 3) {
            sector = 0;
        }

    }

    public void updateLEDsFromState(State state, LEDSector sector, StateStatus stateStatus, double deltaTime, double timestamp) {
        
        if (state.onTime == Double.POSITIVE_INFINITY) {
            // Positive indicates no animaiton
            Color color = state.colors.get(0);
            mCandle.setLEDs((int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255), 0, sector.startLed, sector.ledCount);
            return;
        }

        // Basic oscillation between 0 and onTime then 0 offTime
        if (stateStatus.rising) {
            stateStatus.phase = stateStatus.phase + deltaTime;
            if (stateStatus.phase > state.onTime) {
                stateStatus.rising = false;
                stateStatus.phase = 0;
            }
        } else {
            stateStatus.phase = stateStatus.phase + deltaTime;
            if (stateStatus.phase > state.offTime) {
                stateStatus.rising = true;
                stateStatus.phase = 0;
                if (stateStatus.colorPhase == (state.colors.size() - 1)) {
                    stateStatus.colorPhase = 0;
                } else {
                    stateStatus.colorPhase++;
                }
            }
        }

        // Calculate the proper brightness values
        double percentBrightness = 0;
        if (state.breathing) {
            if (stateStatus.rising) {
                percentBrightness = (stateStatus.phase / state.onTime);
            } else {
                percentBrightness = 1 - (stateStatus.phase / state.offTime);
            }
        } else {
            percentBrightness = stateStatus.rising ? 1 : 0;
        }
        percentBrightness = percentBrightness * sector.brightness;


        int red = 255;
        int blue = 25;
        int green = 0;

        // Calculate R A I N B O W
        if (state.isCycleColors) {
            red   = (int)(Math.sin(5*timestamp + 2) * 127 + 128);
            green = (int)(Math.sin(5*timestamp + 0) * 127 + 128);
            blue  = (int)(Math.sin(5*timestamp + 4) * 127 + 128);
        } else {
            Color color = state.colors.get(stateStatus.colorPhase);
            red = (int)(color.red * percentBrightness * 255);
            green = (int)(color.green * percentBrightness * 255);
            blue = (int)(color.blue * percentBrightness * 255);

        }

        mCandle.setLEDs(red, green, blue, 0, sector.startLed, sector.ledCount);

    }

    public State getState() {
        return mBottomState;
    }

    public State getStripState() {
        return mTopState;
    }

    public void setBottomState(State state) {
        bottomStatus.colorPhase = 0;
        mBottomState = state;
    }

    public void setTopState(State state) {
        topStatus.colorPhase = 0;
        mTopState = state;
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
            }

            @Override
            public void onLoop(double timestamp) {
                updateState();
            }

            @Override
            public void onStop(double timestamp) {
                setBottomState(State.DISABLED);
                setTopState(State.DISABLED);
            }
        });
    }

}