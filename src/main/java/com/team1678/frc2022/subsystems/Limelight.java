package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.loops.ILooper;  

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import com.team254.lib.vision.TargetInfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight extends Subsystem {
    public final static int kDefaultPipeline = 0;
    public final static int kZoomedInPipeline = 1;

    private static Limelight mInstance = null;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private int mLatencyCounter = 0;

    // distance to target
    public Optional<Double> mDistanceToTarget = Optional.empty();

    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "";
        public double kHeight = 0.0;
        public Pose2d kTurretToLens = Pose2d.identity();
        public Rotation2d kHorizontalPlaneToLens = Rotation2d.identity();
    }

    private NetworkTable mNetworkTable;

    private Limelight() {
        mConstants = Constants.VisionConstants.kLimelightConstants;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(mConstants.kTableName);
    }

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }

        return mInstance;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
              //  RobotState.getInstance().resetVision();
              setLed(LedMode.ON);
            }

            @Override
            public void onLoop(double timestamp) {
                final double start = Timer.getFPGATimestamp();
                
                synchronized (this) {
                    if (mSeesTarget) {
                        updateDistanceToTarget();
                    }

                    outputTelemetry();
                    startLogging();
                }

                setLed(LedMode.ON);

                final double end = Timer.getFPGATimestamp();
                mPeriodicIO.dt = end - start;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
                setLed(LedMode.OFF);
            }
        };
        mEnabledLooper.register(mLoop);
    }

    public synchronized boolean limelightOK() {
        return mPeriodicIO.has_comms;
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public boolean has_comms;

        public double dt;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }
    
    private LimelightConstants mConstants = null;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = true;
    private double[] mZeroArray = new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    private List<TargetInfo> mTargets = new ArrayList<>();
    private boolean mSeesTarget = false;

    public double getLensHeight() {
        return mConstants.kHeight;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return mConstants.kHorizontalPlaneToLens;
    }

    public Optional<Double> getDistanceToTarget() {
        return mDistanceToTarget;
    }

    public void updateDistanceToTarget() {
        double goal_theta = Constants.VisionConstants.kLimelightConstants.kHorizontalPlaneToLens.getRadians() + Math.toRadians(mPeriodicIO.yOffset);
        double height_diff = Constants.VisionConstants.kGoalHeight - Constants.VisionConstants.kLimelightConstants.kHeight;

        mDistanceToTarget = Optional.of(height_diff / Math.tan(goal_theta)  + Constants.VisionConstants.kGoalRadius); // add goal radius for offset to center of target
    }

    @Override
    public synchronized void readPeriodicInputs() {
        final double latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.VisionConstants.kImageCaptureLatency;
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);

        if (latency == mPeriodicIO.latency) {
            mLatencyCounter++;
        } else {
            mLatencyCounter = 0;
        }

        mPeriodicIO.latency = latency;
        mPeriodicIO.has_comms = mLatencyCounter < 10;

        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode || mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            // System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {

            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/LIMELIGHT-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean("Limelight Ok", mPeriodicIO.has_comms);
        SmartDashboard.putNumber(mConstants.kName + ": Pipeline Latency (ms)", mPeriodicIO.latency);
        SmartDashboard.putNumber("Limelight dt", mPeriodicIO.dt);

        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mSeesTarget);
        SmartDashboard.putNumber("Limelight Tx: ", mPeriodicIO.xOffset);
        SmartDashboard.putNumber("Limelight Ty: ", mPeriodicIO.yOffset);

        SmartDashboard.putNumber("Distance To Target", mDistanceToTarget.get());

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipeline(int mode) {
        if (mode != mPeriodicIO.pipeline) {
         //   RobotState.getInstance().resetVision();
            mPeriodicIO.pipeline = mode;

            System.out.println(mPeriodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    public synchronized boolean hasTarget() {
        return mSeesTarget;
    }

    /**
     * @return two targets that make up one hatch/port or null if less than two
     *         targets are found
     */
    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = new ArrayList<TargetInfo>(); //getRawTargetInfos();
        targets.add(new TargetInfo(Math.tan(Math.toRadians(-mPeriodicIO.xOffset)), Math.tan(Math.toRadians(mPeriodicIO.yOffset))));
        if (hasTarget() && targets != null) {
            return targets;
        }

        return null;
    }

    private synchronized List<TargetInfo> getRawTargetInfos() {
        List<double[]> corners = getTopCorners();
        if (corners == null) {
            return null;
        }

        double slope = 1.0;
        if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Util.kEpsilon) {
            slope = (corners.get(1)[1] - corners.get(0)[1]) / (corners.get(1)[0] - corners.get(0)[0]);
        }

        mTargets.clear();
        for (int i = 0; i < 2; ++i) {
            // Average of y and z;
            double y_pixels = corners.get(i)[0];
            double z_pixels = corners.get(i)[1];

            // Redefine to robot frame of reference.
            double nY = -((y_pixels - 160.0) / 160.0);
            double nZ = -((z_pixels - 120.0) / 120.0);

            double y = Constants.VisionConstants.kVPW / 2 * nY;
            double z = Constants.VisionConstants.kVPH / 2 * nZ;

            TargetInfo target = new TargetInfo(y, z);
            target.setSkew(slope);
            mTargets.add(target);
        }

        return mTargets;
    }

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private List<double[]> getTopCorners() {
        double[] xyCorners = mNetworkTable.getEntry("tcornxy").getDoubleArray(mZeroArray);
        ArrayList<Double> xCorners = new ArrayList<>();
        ArrayList<Double> yCorners = new ArrayList<>();

        for (int i = 0; i < xyCorners.length; i++) {
            if (i % 2 == 0) {
                xCorners.add(xyCorners[i]);
            } else {
                yCorners.add(xyCorners[i]);
            }
        }

        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        double[] xCornersArray = xCorners.stream().mapToDouble(Double::doubleValue).toArray();
        double[] yCornersArray = yCorners.stream().mapToDouble(Double::doubleValue).toArray();

        // something went wrong

        if (!mSeesTarget || Arrays.equals(xCornersArray, mZeroArray) || Arrays.equals(yCornersArray, mZeroArray)
                || xCornersArray.length < 4 || xCornersArray.length != yCornersArray.length) {
            return null;
        }

        return extractTopCornersFromBoundingBoxes(xCornersArray, yCornersArray);
    }

    private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::x);
    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);

        if (corners.size() > 4) {
            for (int i = 0; i < corners.size() - 4; i++) {
                corners.remove(1 + i);
            }
        }

        List<Translation2d> left = corners.subList(0, 2);
        List<Translation2d> right = corners.subList(2, 4);

        left.sort(ySort);
        right.sort(ySort);

        List<Translation2d> leftTop = left.subList(0, 2);
        List<Translation2d> rightTop = right.subList(0, 2);

        leftTop.sort(xSort);
        rightTop.sort(xSort);

        Translation2d leftCorner = leftTop.get(0);
        Translation2d rightCorner = rightTop.get(1);

        return List.of(new double[] { leftCorner.x(), leftCorner.y() },
                new double[] { rightCorner.x(), rightCorner.y() });
    }

    public double getLatency() {
        return mPeriodicIO.latency;
    }

    public double getDt() {
        return mPeriodicIO.dt;
    }

    public double[] getOffset() {
        return new double[] {mPeriodicIO.xOffset, mPeriodicIO.yOffset};
    }

    public boolean getOnTarget() {
        return Util.epsilonEquals(mPeriodicIO.xOffset, 0., Constants.VisionAlignConstants.kVisionAlignEpsilon);
    }
}
