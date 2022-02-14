package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends Subsystem {

    private final CANdle mCandle = new CANdle(Ports.CANDLE);

    private static LEDs mInstance;

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    private double timestamp = 0.0;

    private final boolean mUseSmartdash = false; // if we want to manual control lights using shuffleboard
    private boolean mLastUpdatedTop = false; // alternate updating sections to avoid overrunning the CANdle
    private boolean mIdle = true; // run animation if disabled

    // led sections
    private LEDStatus mTopStatus = new LEDStatus(39, 68);  
    private LEDStatus mBottomStatus = new LEDStatus(8, 39);

    // shuffleboard selectors
    private SendableChooser<State> mTopStateChooser;
    private SendableChooser<State> mBottomStateChooser;

    // animation to run when disabled
    private Animation mDisableAnimation = new ColorFlowAnimation(255, 18, 143, 0, 0.7, 68, Direction.Forward);

    // led states
    public enum State{
        OFF("OFF", Double.POSITIVE_INFINITY, Color.off()),
        EMERGENCY("EMERGENCY", 0.05, new Color(255, 0, 0), Color.off()),

        SOLID_RED("SOLID_RED", Double.POSITIVE_INFINITY, new Color(255, 0, 0)),
        SOLID_PINK("SOLID_PINK", Double.POSITIVE_INFINITY, new Color(255, 18, 143)),
        SOLID_GREEN("SOLID_GREEN", Double.POSITIVE_INFINITY, new Color(0, 255, 8)),
        SOLID_PURPLE("SOLID_PURPLE", Double.POSITIVE_INFINITY, new Color(196, 18, 255)),
        SOLID_ORANGE("SOLID_ORANGE", Double.POSITIVE_INFINITY, new Color(255, 53, 13)),
        SOLID_YELLOW("SOLID_YELLOW", Double.POSITIVE_INFINITY, new Color(243, 255, 18)),
        SOLID_CYAN("SOLID_CYAN", Double.POSITIVE_INFINITY, new Color(18, 239, 255)),

        FLASHING_RED("FLASHING_RED", 0.05, new Color(255, 0, 0), Color.off()),
        FLASHING_PINK("FLASHING_PINK", 0.05, new Color(255, 20, 0), Color.off()),
        FLASHING_GREEN("FLASHING_GREEN", 0.05, new Color(0, 255, 0), Color.off()),
        FLASHING_PURPLE("FLASHING_PURPLE", 0.05, new Color(255, 0, 255), Color.off()),
        FLASHING_ORANGE("FLASHING_ORANGE", 0.05, new Color(255, 53, 0), Color.off()),
        FLASHING_YELLOW("FLASHING_YELLOW", 0.05, new Color(255, 255, 0), Color.off()),
        FLASHING_CYAN("FLASHING_CYAN", 0.05, new Color(0, 255, 255), Color.off());

        Color[] colors; // array of colors to iterate over
        double interval; // time in seconds between states
        String name; // name of state

        private State(String name, double interval, Color... colors) {
            this.colors = colors;
            this.interval = interval;
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }

    public LEDs() {
        configureCandle(); // set CTRE configurations for CANdle

        // create sendable choosers for shuffleboard
        if (mUseSmartdash) {
            mTopStateChooser = new SendableChooser<>();
            mBottomStateChooser = new SendableChooser<>();
            for (State state : State.values()) {
                mTopStateChooser.addOption(state.getName(),  state);
                mBottomStateChooser.addOption(state.getName(),  state);
            }
            mTopStateChooser.setDefaultOption("OFF", State.OFF);
            mBottomStateChooser.setDefaultOption("OFF", State.OFF);
            SmartDashboard.putData("Top LEDs", mTopStateChooser);
            SmartDashboard.putData("Bottom LEDs", mBottomStateChooser);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mIdle = false;
                applyStates(State.OFF, State.OFF); 
            }

            @Override
            public void onLoop(double timestamp) {
                
            }

            @Override
            public void onStop(double timestamp) {
                mIdle = true;
                mTopStatus.reset();
                mBottomStatus.reset();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        timestamp = Timer.getFPGATimestamp();
        outputTelemtry();
        if (mUseSmartdash) {
            applyStates(mTopStateChooser.getSelected(), mBottomStateChooser.getSelected()); // pull states from smartdash
        }
    }

    public void updateState() {
        if (!mIdle) {
            // alternate updating each section of leds to avoid overrunning the CANdle
            if (mLastUpdatedTop) {
                updateBottomLeds();
                mLastUpdatedTop = false;
            } else {
                updateTopLeds();
                mLastUpdatedTop = true;
            }
        } else {
            mCandle.animate(mDisableAnimation);
        }

    }

    private void updateBottomLeds() {
        // cycle to next color
        if (mBottomStatus.state.interval != Double.POSITIVE_INFINITY) {
            if (timestamp - mBottomStatus.lastSwitchTime >= mBottomStatus.state.interval) {
                mBottomStatus.nextColor();
                mBottomStatus.lastSwitchTime = timestamp;
            }
        }
        Color bottomColor = mBottomStatus.getWantedColor();

        mCandle.setLEDs(bottomColor.r, bottomColor.g, bottomColor.b, 0, mBottomStatus.startIDx, mBottomStatus.LEDCount);
    }

    private void updateTopLeds() {
        // cycle to next color
        if (mTopStatus.state.interval != Double.POSITIVE_INFINITY) {
            if (timestamp - mTopStatus.lastSwitchTime >= mTopStatus.state.interval) {
                mTopStatus.nextColor();
                mTopStatus.lastSwitchTime = timestamp;
            }
        }
        Color topColor = mTopStatus.getWantedColor();

        mCandle.setLEDs(topColor.r, topColor.g, topColor.b, 0, mTopStatus.startIDx, mTopStatus.LEDCount);
    }

    public void applyStates(State topState, State bottomState) {
        mTopStatus.setState(topState);
        mBottomStatus.setState(bottomState);
    }

    public void applyTopState(State state) {
        mTopStatus.setState(state);
    }

    public void applyBottomState(State state) {
        mBottomStatus.setState(state);
    }

    private void configureCandle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.05;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        mCandle.configAllSettings(configAll, Constants.kLongCANTimeoutMs);
        mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public State getTopState() {
        return mTopStatus.state;
    }

    public State getBottomState() {
        return mBottomStatus.state;
    }

    public void setIdle(boolean isIdle) {
        mIdle = isIdle;
    }

    private void outputTelemtry() {
        SmartDashboard.putString("Top LED Status", getTopState().name);
        SmartDashboard.putString("Bottom LED Status", getBottomState().name);

        SmartDashboard.putString("Top LED Colors", mTopStatus.getWantedColor().toString());
        SmartDashboard.putString("Bottom LED Colors", mBottomStatus.getWantedColor().toString());
        
        SmartDashboard.putBoolean("Idle Animating", mIdle);
    }

    // class for holding information about each section
    private class LEDStatus {
        private State state = State.OFF;
        private double lastSwitchTime = 0.0;
        private int colorIndex = 0;
        private int startIDx, LEDCount;

        public LEDStatus(int startIndex, int endIndex) {
            startIDx = startIndex;
            LEDCount = endIndex - startIndex;
        }

        public void setState(State wantedState) {
            if (wantedState != state) {
                colorIndex = 0;
                lastSwitchTime = Timer.getFPGATimestamp();
                state = wantedState;
            }
        }

        public Color getWantedColor() {
            return state.colors[colorIndex];
        }

        public void nextColor() {
            if (colorIndex == state.colors.length - 1) {
                colorIndex = 0;
            } else {
                colorIndex++;
            }
        }

        public void reset() {
            state = State.OFF;
            lastSwitchTime = 0.0;
            colorIndex = 0;
        }
    }

    // class to hold rgb values for a color
    private static class Color {
        public int r;
        public int g;
        public int b;

        public Color(int red, int green, int blue) {
            r = red;
            g = green;
            b = blue;
        }

        public static Color off() {
            return new Color(0, 0, 0);
        }

        @Override
        public String toString() {
            return "(" + r + "," + g + "," + b + ")";
        }
    }
}
