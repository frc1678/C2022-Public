package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.motion.MotionProfileConstraints;
import com.team254.lib.motion.MotionProfileGoal;
import com.team254.lib.motion.MotionState;
import com.team254.lib.motion.SetpointGenerator;
import com.team254.lib.motion.SetpointGenerator.Setpoint;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 */
public abstract class ServoMotorSubsystem extends Subsystem {
    private static final int kMotionProfileSlot = 0;
    private static final int kPositionPIDSlot = 1;

    // Recommend initializing in a static block!
    public static class TalonFXConstants {
        public int id = -1;
        public boolean invert_motor = false;
        public boolean invert_sensor_phase = false;
    }

    // Recommend initializing in a static block!
    public static class ServoMotorSubsystemConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public TalonFXConstants kMasterConstants = new TalonFXConstants();
        public TalonFXConstants[] kSlaveConstants = new TalonFXConstants[0];

        public double kHomePosition = 0.0; // Units
        public double kTicksPerUnitDistance = 1.0;
        public double kKp = 0; // Raw output / raw error
        public double kKi = 0; // Raw output / sum of raw error
        public double kKd = 0; // Raw output / (err - prevErr)
        public double kKf = 0; // Raw output / velocity in ticks/100ms
        public double kKa = 0; // Raw output / accel in (ticks/100ms) / s
        public double kMaxIntegralAccumulator = 0;
        public int kIZone = 0; // Ticks
        public int kDeadband = 0; // Ticks

        public double kPositionKp = 0;
        public double kPositionKi = 0;
        public double kPositionKd = 0;
        public double kPositionKf = 0;
        public double kPositionMaxIntegralAccumulator = 0;
        public int kPositionIZone = 0; // Ticks
        public int kPositionDeadband = 0; // Ticks

        public int kCruiseVelocity = 0; // Ticks / 100ms
        public int kAcceleration = 0; // Ticks / 100ms / s
        public double kRampRate = 0.0; // s
        public int kContinuousCurrentLimit = 20; // amps
        public int kPeakCurrentLimit = 60; // amps
        public int kPeakCurrentDuration = 200; // milliseconds
        public double kMaxVoltage = 12.0;

        public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
        public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

        public int kStastusFrame8UpdateRate = 1000;
        public boolean kRecoverPositionOnReset = false;
    }

    protected final ServoMotorSubsystemConstants mConstants;
    protected final TalonFX mMaster;
    protected final TalonFX[] mSlaves;

    protected final int mForwardSoftLimitTicks;
    protected final int mReverseSoftLimitTicks;

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 300, 700,
            1);
    public SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(false, 40, 100,
            1);

    protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
        mConstants = constants;
        mMaster = TalonFXFactory.createDefaultTalon(mConstants.kMasterConstants.id);
        mSlaves = new TalonFX[mConstants.kSlaveConstants.length];
        mMaster.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);
        mMaster.configSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT);

        TalonUtil.checkError(mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kLongCANTimeoutMs), mConstants.kName + ": Could not detect encoder: ");

        mForwardSoftLimitTicks = (int) ((mConstants.kMaxUnitsLimit - mConstants.kHomePosition)
                * mConstants.kTicksPerUnitDistance);
        TalonUtil.checkError(
                mMaster.configForwardSoftLimitThreshold(mForwardSoftLimitTicks, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set forward soft limit: ");

        TalonUtil.checkError(mMaster.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not enable forward soft limit: ");

        mReverseSoftLimitTicks = (int) ((mConstants.kMinUnitsLimit - mConstants.kHomePosition)
                * mConstants.kTicksPerUnitDistance);
        TalonUtil.checkError(
                mMaster.configReverseSoftLimitThreshold(mReverseSoftLimitTicks, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set reverse soft limit: ");

        TalonUtil.checkError(mMaster.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not enable reverse soft limit: ");

        TalonUtil.checkError(mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set voltage compensation saturation: ");

        TalonUtil.checkError(mMaster.config_kP(kMotionProfileSlot, mConstants.kKp, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kP: ");

        TalonUtil.checkError(mMaster.config_kI(kMotionProfileSlot, mConstants.kKi, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kI: ");

        TalonUtil.checkError(mMaster.config_kD(kMotionProfileSlot, mConstants.kKd, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kD: ");

        TalonUtil.checkError(mMaster.config_kF(kMotionProfileSlot, mConstants.kKf, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set kF: ");

        TalonUtil
                .checkError(mMaster.configMaxIntegralAccumulator(kMotionProfileSlot, mConstants.kMaxIntegralAccumulator,
                        Constants.kLongCANTimeoutMs), mConstants.kName + ": Could not set max integral: ");

        TalonUtil.checkError(
                mMaster.config_IntegralZone(kMotionProfileSlot, mConstants.kIZone, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set i zone: ");

        TalonUtil.checkError(
                mMaster.configAllowableClosedloopError(kMotionProfileSlot, mConstants.kDeadband,
                        Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set deadband: ");

        TalonUtil.checkError(mMaster.config_kP(kPositionPIDSlot, mConstants.kPositionKp, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kP: ");

        TalonUtil.checkError(mMaster.config_kI(kPositionPIDSlot, mConstants.kPositionKi, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kI: ");

        TalonUtil.checkError(mMaster.config_kD(kPositionPIDSlot, mConstants.kPositionKd, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": could not set kD: ");

        TalonUtil.checkError(mMaster.config_kF(kPositionPIDSlot, mConstants.kPositionKf, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set kF: ");

        TalonUtil.checkError(
                mMaster.configMaxIntegralAccumulator(kPositionPIDSlot, mConstants.kPositionMaxIntegralAccumulator,
                        Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set max integral: ");

        TalonUtil.checkError(
                mMaster.config_IntegralZone(kPositionPIDSlot, mConstants.kPositionIZone, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set i zone: ");

        TalonUtil.checkError(
                mMaster.configAllowableClosedloopError(kPositionPIDSlot, mConstants.kPositionDeadband,
                        Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set deadband: ");

        TalonUtil.checkError(
                mMaster.configMotionCruiseVelocity(mConstants.kCruiseVelocity, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set cruise velocity: ");

        TalonUtil.checkError(mMaster.configMotionAcceleration(mConstants.kAcceleration, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set acceleration: ");

        TalonUtil.checkError(mMaster.configOpenloopRamp(mConstants.kRampRate, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set voltage ramp rate: ");

        TalonUtil.checkError(mMaster.configClosedloopRamp(mConstants.kRampRate, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set closed loop ramp rate: ");

        TalonUtil.checkError(
                mMaster.configStatorCurrentLimit(STATOR_CURRENT_LIMIT, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set stator current limit.");

        TalonUtil.checkError(
                mMaster.configSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set supply current limit.");

        mMaster.configVoltageMeasurementFilter(8);

        TalonUtil.checkError(
                mMaster.configVoltageCompSaturation(mConstants.kMaxVoltage, Constants.kLongCANTimeoutMs),
                mConstants.kName + ": Could not set voltage comp saturation.");
        mMaster.enableVoltageCompensation(true);

        mMaster.setInverted(mConstants.kMasterConstants.invert_motor);
        mMaster.setSensorPhase(mConstants.kMasterConstants.invert_sensor_phase);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, mConstants.kStastusFrame8UpdateRate, 20);

        // Start with kMotionProfileSlot.
        mMaster.selectProfileSlot(kMotionProfileSlot, 0);

        for (int i = 0; i < mSlaves.length; ++i) {
            mSlaves[i] = TalonFXFactory.createPermanentSlaveTalon(mConstants.kSlaveConstants[i].id,
                    mConstants.kMasterConstants.id);
            mSlaves[i].setInverted(mConstants.kSlaveConstants[i].invert_motor);
            mSlaves[i].setNeutralMode(NeutralMode.Brake);
            mSlaves[i].follow(mMaster);
        }

        // The accel term can re-use the velocity unit conversion because both input and
        // output units are per second.
        mMotionProfileConstraints = new MotionProfileConstraints(
                ticksPer100msToUnitsPerSecond(mConstants.kCruiseVelocity),
                ticksPer100msToUnitsPerSecond(mConstants.kAcceleration));

        // Send a neutral command.
        stop();
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public int position_ticks;
        public double position_units;
        public int velocity_ticks_per_100ms;
        public int active_trajectory_position; // ticks
        public int active_trajectory_velocity; // ticks/100ms
        public double active_trajectory_acceleration; // ticks/100ms/s
        public double output_percent;
        public double output_voltage;
        public double master_supply_current;
        public double master_stator_current;
        public double error_ticks;
        public int encoder_wraps;
        public double absolute_pulse_offset;
        public double absolute_pulse_position;
        public double absolute_pulse_position_modded;
        public boolean reset_occured;

        public double dt;

        // OUTPUTS
        public double demand;
        public double feedforward;
    }

    protected enum ControlState {
        OPEN_LOOP, MOTION_MAGIC, POSITION_PID, MOTION_PROFILING
    }

    protected PeriodicIO mPeriodicIO = new PeriodicIO();
    protected ControlState mControlState = ControlState.OPEN_LOOP;
    protected boolean mHasBeenZeroed = false;
    protected StickyFaults mFaults = new StickyFaults();
    protected SetpointGenerator mSetpointGenerator = new SetpointGenerator();
    protected MotionProfileConstraints mMotionProfileConstraints;

    public synchronized void setNeutralMode(NeutralMode mode) {
        mMaster.setNeutralMode(mode);
        for (int i = 0; i < mSlaves.length; ++i) {
            mSlaves[i].setNeutralMode(mode);
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        if (mMaster.hasResetOccurred()) {
            DriverStation.reportError(mConstants.kName + ": Talon Reset! ", false);
            mPeriodicIO.reset_occured = true;
            return;
        } else {
            mPeriodicIO.reset_occured = false;
        }

        mMaster.getStickyFaults(mFaults);
        if (mFaults.hasAnyFault()) {
            DriverStation.reportError(mConstants.kName + ": Talon Fault! " + mFaults.toString(), false);
            mMaster.clearStickyFaults(0);
        }
        if (mMaster.getControlMode() == ControlMode.MotionMagic) {
            mPeriodicIO.active_trajectory_position = (int) mMaster.getActiveTrajectoryPosition();

            if (mPeriodicIO.active_trajectory_position < mReverseSoftLimitTicks) {
                DriverStation.reportError(mConstants.kName + ": Active trajectory past reverse soft limit!", false);
            } else if (mPeriodicIO.active_trajectory_position > mForwardSoftLimitTicks) {
                DriverStation.reportError(mConstants.kName + ": Active trajectory past forward soft limit!", false);
            }
            final int newVel = (int) mMaster.getActiveTrajectoryVelocity();
            if (Util.epsilonEquals(newVel, mConstants.kCruiseVelocity, Math.max(1, mConstants.kDeadband)) || Util
                    .epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, Math.max(1, mConstants.kDeadband))) {
                // Mechanism is ~constant velocity.
                mPeriodicIO.active_trajectory_acceleration = 0.0;
            } else {
                // Mechanism is accelerating.
                mPeriodicIO.active_trajectory_acceleration = Math
                        .signum(newVel - mPeriodicIO.active_trajectory_velocity) * mConstants.kAcceleration;
            }
            mPeriodicIO.active_trajectory_velocity = newVel;
        } else {
            mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
            mPeriodicIO.active_trajectory_velocity = 0;
            mPeriodicIO.active_trajectory_acceleration = 0.0;
        }
        if (mMaster.getControlMode() == ControlMode.Position) {
            mPeriodicIO.error_ticks = mMaster.getClosedLoopError(0);
        } else {
            mPeriodicIO.error_ticks = 0;
        }
        mPeriodicIO.master_supply_current = mMaster.getSupplyCurrent();
        mPeriodicIO.master_stator_current = mMaster.getStatorCurrent();
        mPeriodicIO.output_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.output_percent = mMaster.getMotorOutputPercent();
        mPeriodicIO.position_ticks = (int) mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.position_units = ticksToHomedUnits(mPeriodicIO.position_ticks);
        mPeriodicIO.velocity_ticks_per_100ms = (int) mMaster.getSelectedSensorVelocity(0);

        if (mConstants.kRecoverPositionOnReset) {
            mPeriodicIO.absolute_pulse_position = mMaster.getSensorCollection().getIntegratedSensorAbsolutePosition();
            mPeriodicIO.absolute_pulse_position_modded = mPeriodicIO.absolute_pulse_position % 4096;
            if (mPeriodicIO.absolute_pulse_position_modded < 0) {
                mPeriodicIO.absolute_pulse_position_modded += 4096;
            }

            double estimated_pulse_pos = ((mConstants.kMasterConstants.invert_sensor_phase ? -1 : 1)
                    * mPeriodicIO.position_ticks) + mPeriodicIO.absolute_pulse_offset;
            int new_wraps = (int) Math.floor(estimated_pulse_pos / 4096.0);
            // Only set this when we are really sure its a valid change
            if (Math.abs(mPeriodicIO.encoder_wraps - new_wraps) <= 1) {
                mPeriodicIO.encoder_wraps = new_wraps;
            }
        }
    }

    protected double getAbsoluteEncoderRawPosition(double pulseWidthPosition) {
        double abs_raw_no_rollover = mMaster.getSensorCollection().getIntegratedSensorAbsolutePosition();
        double abs_raw_with_rollover = abs_raw_no_rollover % 4096;
        return abs_raw_with_rollover + (abs_raw_with_rollover < 0 ? abs_raw_with_rollover + 4096 : 0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mControlState == ControlState.MOTION_MAGIC) {
            mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.feedforward);
        } else if (mControlState == ControlState.POSITION_PID || mControlState == ControlState.MOTION_PROFILING) {
            mMaster.set(ControlMode.Position, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.feedforward);
        } else {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.feedforward);
        }
    }

    public synchronized void handleMasterReset(boolean reset) {
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                /*
                 * if (mCSVWriter == null) {
                 * mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/"
                 * + mConstants.kName.replaceAll("[^A-Za-z0-9]+", "").toUpperCase() +
                 * "-LOGS.csv",
                 * PeriodicIO.class);
                 * }
                 */
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();
                if (mPeriodicIO.reset_occured) {
                    System.out.println(mConstants.kName + ": Master Talon reset occurred; resetting frame rates.");
                    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
                    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
                    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                            mConstants.kStastusFrame8UpdateRate, 20);

                    // Reset encoder position to estimated position from absolute encoder
                    if (mConstants.kRecoverPositionOnReset) {
                        mMaster.setSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0,
                                Constants.kCANTimeoutMs);
                    }
                }
                handleMasterReset(mPeriodicIO.reset_occured);
                for (TalonFX slave : mSlaves) {
                    if (slave.hasResetOccurred()) {
                        System.out.println(mConstants.kName + ": Slave Talon reset occurred");
                    }
                }
                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized double getPositionTicks() {
        return mPeriodicIO.position_ticks;
    }

    // In "Units"
    public synchronized double getPosition() {
        return ticksToHomedUnits(mPeriodicIO.position_ticks);
    }

    // In "Units per second"
    public synchronized double getVelocity() {
        return ticksToUnits(mPeriodicIO.velocity_ticks_per_100ms) * 10.0;
    }

    public synchronized boolean hasFinishedTrajectory() {
        if (Util.epsilonEquals(mPeriodicIO.active_trajectory_position, ticksToUnits(getSetpoint()),
                Math.max(1, mConstants.kDeadband))) {
            return true;
        }
        return false;
    }

    public synchronized double getSetpoint() {
        return (mControlState == ControlState.MOTION_MAGIC ||
                mControlState == ControlState.POSITION_PID) ? ticksToUnits(mPeriodicIO.demand) : Double.NaN;
    }

    public synchronized void setSetpointMotionMagic(double units, double feedforward_v) {
        mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
        mPeriodicIO.feedforward = unitsPerSecondToTicksPer100ms(feedforward_v)
                * (mConstants.kKf + mConstants.kKd / 100.0) / 1023.0;
        if (mControlState != ControlState.MOTION_MAGIC) {
            mMaster.selectProfileSlot(kMotionProfileSlot, 0);
            mControlState = ControlState.MOTION_MAGIC;
        }
    }

    public synchronized void setSetpointMotionMagic(double units) {
        setSetpointMotionMagic(units, 0.0);
    }

    public synchronized void setSetpointPositionPID(double units, double feedforward_v) {
        mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(units));
        double feedforward_ticks_per_100ms = unitsPerSecondToTicksPer100ms(feedforward_v);
        mPeriodicIO.feedforward = feedforward_ticks_per_100ms * (mConstants.kKf + mConstants.kKd / 100.0) / 1023.0;
        if (mControlState != ControlState.POSITION_PID) {
            mMaster.selectProfileSlot(kPositionPIDSlot, 0);
            mControlState = ControlState.POSITION_PID;
        }
    }

    public synchronized void setGoalMotionProfiling(MotionProfileGoal goal, double feedforward_v) {
        if (mControlState != ControlState.MOTION_PROFILING) {
            mMaster.selectProfileSlot(kPositionPIDSlot, 0);
            mControlState = ControlState.MOTION_PROFILING;
            mSetpointGenerator.reset();
        }
        MotionState cur_state = new MotionState(mPeriodicIO.timestamp, mPeriodicIO.position_units,
                ticksPer100msToUnitsPerSecond(mPeriodicIO.velocity_ticks_per_100ms), 0.0);
        Setpoint setpoint = mSetpointGenerator.getSetpoint(mMotionProfileConstraints, goal, cur_state,
                mPeriodicIO.timestamp + Constants.kLooperDt);
        mPeriodicIO.demand = constrainTicks(homeAwareUnitsToTicks(setpoint.motion_state.pos()));
        mPeriodicIO.feedforward = ((feedforward_v + setpoint.motion_state.vel()) * mConstants.kKf
                + setpoint.motion_state.acc() * mConstants.kKa) / 1023.0;
    }

    protected double ticksToUnits(double ticks) {
        return ticks / mConstants.kTicksPerUnitDistance;
    }

    protected double ticksToHomedUnits(double ticks) {
        double val = ticksToUnits(ticks);
        return val + mConstants.kHomePosition;
    }

    protected double unitsToTicks(double units) {
        return units * mConstants.kTicksPerUnitDistance;
    }

    protected double homeAwareUnitsToTicks(double units) {
        return unitsToTicks(units - mConstants.kHomePosition);
    }

    protected double constrainTicks(double ticks) {
        return Util.limit(ticks, mReverseSoftLimitTicks, mForwardSoftLimitTicks);
    }

    protected double ticksPer100msToUnitsPerSecond(double ticks_per_100ms) {
        return ticksToUnits(ticks_per_100ms) * 10.0;
    }

    protected double unitsPerSecondToTicksPer100ms(double units_per_second) {
        return unitsToTicks(units_per_second) / 10.0;
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }
    }

    public synchronized double getActiveTrajectoryUnits() {
        return ticksToHomedUnits(mPeriodicIO.active_trajectory_position);
    }

    public synchronized double getActiveTrajectoryVelocityUnitsPerSec() {
        return ticksPer100msToUnitsPerSecond(mPeriodicIO.active_trajectory_velocity);
    }

    public synchronized double getPredictedPositionUnits(double lookahead_secs) {
        if (mMaster.getControlMode() != ControlMode.MotionMagic) {
            return getPosition();
        }

        double predicted_units = ticksToHomedUnits(mPeriodicIO.active_trajectory_position +
                lookahead_secs * mPeriodicIO.active_trajectory_velocity +
                0.5 * mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs);
        if (mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position) {
            return Math.min(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
        } else {
            return Math.max(predicted_units, ticksToHomedUnits(mPeriodicIO.demand));
        }
    }

    public boolean atHomingLocation() {
        return false;
    }

    public synchronized void resetIfAtLimit() {
        if (atHomingLocation()) {
            zeroSensors();
        }
    }

    @Override
    public synchronized void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        mPeriodicIO.absolute_pulse_offset = getAbsoluteEncoderRawPosition(
                mMaster.getSensorCollection().getIntegratedSensorAbsolutePosition());
        mHasBeenZeroed = true;
    }

    public synchronized boolean hasBeenZeroed() {
        return mHasBeenZeroed;
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public int estimateSensorPositionFromAbsolute() {
        double estimated_pulse_pos = (mPeriodicIO.encoder_wraps * 4096) + mPeriodicIO.absolute_pulse_position_modded;
        double estimate_position_ticks = (mConstants.kMasterConstants.invert_sensor_phase ? -1 : 1)
                * (estimated_pulse_pos - mPeriodicIO.absolute_pulse_offset);
        return (int) estimate_position_ticks;
    }

}