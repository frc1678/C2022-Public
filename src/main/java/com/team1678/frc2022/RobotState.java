package com.team1678.frc2022;

import com.team1678.frc2022.subsystems.Limelight;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAveragePose2d;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.vision.TargetInfo;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

public class RobotState {
    private static RobotState mInstance;
    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 1;
    public static final Pose2d kDefaultFieldRelativeGoalLocation = new Pose2d(8.2296, 4.12155, new Rotation2d());
    public static final Pose2d kFiveBallStartingLocation = new Pose2d(8.597, 1.529, new Rotation2d());

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Camera frame: origin is the center of the Limelight imager relative to the
     * turret.
     *
     * 4. Goal frame: origin is the center of the vision target, facing outwards
     * along the normal. Also note that there can be multiple goal frames.
     *
     * As a kinematic chain with 4 frames, there are 2 transforms of interest:
     *
     * 1. Field-to-robot: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Camera-to-goal: Measured by the vision system.
     * 
     */

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private Pose2d vehicle_velocity_predicted_;
    private MovingAveragePose2d vehicle_velocity_predicted_filtered_;
    private Pose2d vehicle_velocity_measured_;
    private MovingAveragePose2d vehicle_velocity_measured_filtered_;
    private double distance_driven_;

    private GoalTracker goal_tracker_ = new GoalTracker();

    List<Translation2d> mCameraToVisionTargetPoses = new ArrayList<>();

    private RobotState() {
        reset(0.0, Pose2d.identity());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Pose2d.identity();
        vehicle_velocity_predicted_filtered_ = new MovingAveragePose2d(50);
        vehicle_velocity_measured_ = Pose2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAveragePose2d(50);
        distance_driven_ = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    public synchronized void reset(Pose2d pose) {
        reset(Timer.getFPGATimestamp(), pose);
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return new Pose2d(
            getLatestFieldToVehicle().getValue()
                .transformBy(getSmoothedPredictedVelocity().scaled(-lookahead_time)).getTranslation(), new Rotation2d());
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Pose2d displacement, Pose2d measured_velocity,
            Pose2d predicted_velocity) {

        distance_driven_ += displacement.getTranslation().norm();
        addFieldToVehicleObservation(timestamp, new Pose2d(
            getLatestFieldToVehicle().getValue().getTranslation().translateBy(displacement.getTranslation()), 
            getLatestFieldToVehicle().getValue().getRotation().rotateBy(displacement.getRotation())));

        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_predicted_ = predicted_velocity;

        // add measured velocity to moving average array for filter
        vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        // add predicted velocity to moving average array for filter
        vehicle_velocity_predicted_filtered_.add(vehicle_velocity_predicted_);
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized Pose2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Pose2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Pose2d getSmoothedMeasuredVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized Pose2d getSmoothedPredictedVelocity() {
        return vehicle_velocity_predicted_filtered_.getAverage();
    }

    public synchronized void resetVision() {
        goal_tracker_.reset();
    }

    private Translation2d getCameraToVisionTargetTranslation(TargetInfo target, Limelight source) {
        return getCameraToVisionTargetTranslation(target, source.getLensHeight(), source.getHorizontalPlaneToLens());
   }

    public static Translation2d getCameraToVisionTargetTranslation(TargetInfo target, double cameraHeight, Rotation2d cameraPitch) {
        return getCameraToVisionTargetTranslation(target, cameraHeight, cameraPitch, Constants.VisionConstants.kGoalHeight);
    }

    private static Translation2d getCameraToVisionTargetTranslation(TargetInfo target, double cameraHeight, Rotation2d cameraPitch, double targetCornerHeight) {
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(cameraPitch);
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double differential_height = targetCornerHeight - cameraHeight;
        if ((z > 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos() + Constants.VisionConstants.kGoalRadius, distance * angle.sin());
        }
        return null;
    }

    private void updateGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetPoses, GoalTracker tracker,
            Limelight source) {
        if (cameraToVisionTargetPoses.size() != 1
                || cameraToVisionTargetPoses.get(0) == null /*
                                                             * || cameraToVisionTargetPoses.get(1) == null
                                                             */)
            return;
        Pose2d cameraToVisionTarget = Pose2d.fromTranslation(cameraToVisionTargetPoses.get(0));

        SmartDashboard.putString("camera to vision target", cameraToVisionTarget.toString());

        Pose2d fieldToVisionTarget = getFieldToVehicle(timestamp).transformBy(cameraToVisionTarget.inverse());

        SmartDashboard.putString("field to vision target", fieldToVisionTarget.toString());

        tracker.update(timestamp, List.of(new Pose2d(fieldToVisionTarget.getTranslation(), Rotation2d.identity())));
    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations, Limelight source) {
        List<Translation2d> cameraToVisionTargetTranslations = new ArrayList<>();

        if (observations == null || observations.isEmpty()) {
            goal_tracker_.maybePruneTracks();
            return;
        }

        for (TargetInfo target : observations) {
            cameraToVisionTargetTranslations.add(getCameraToVisionTargetTranslation(target, source));
        }

        updateGoalTracker(timestamp, cameraToVisionTargetTranslations, goal_tracker_, source);
    }

    // use known field target orientations to compensate for inaccuracy, assumes
    // robot starts pointing directly away
    // from and perpendicular to alliance wall
    private final double[] kPossibleTargetNormals = { 0.0, 90.0, 180.0, 270.0 };

    public synchronized Pose2d getFieldToVisionTarget() {
        GoalTracker tracker = goal_tracker_;

        if (!tracker.hasTracks()) {
            return null;
        }

        Pose2d fieldToTarget = tracker.getTracks().get(0).field_to_target;

        double normalPositive = (fieldToTarget.getRotation().getDegrees() + 360) % 360;
        double normalClamped = kPossibleTargetNormals[0];
        for (double possible : kPossibleTargetNormals) {
            if (Math.abs(normalPositive - possible) < Math.abs(normalPositive - normalClamped)) {
                normalClamped = possible;
            }
        }

        return new Pose2d(fieldToTarget.getTranslation(), Rotation2d.fromDegrees(normalClamped));
    }

    public synchronized Pose2d getVehicleToVisionTarget(double timestamp) {
        Pose2d fieldToVisionTarget = getFieldToVisionTarget();

        if (fieldToVisionTarget == null) {
            return null;
        }

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToVisionTarget);
    }

    public synchronized Optional<AimingParameters> getDefaultAimingParameters() {
        double timestamp = Timer.getFPGATimestamp();

        Pose2d vehicleToGoal = getFieldToVehicle(timestamp).inverse().transformBy(kDefaultFieldRelativeGoalLocation);
        vehicleToGoal = new Pose2d(vehicleToGoal.getTranslation().rotateBy(getFieldToVehicle(timestamp).getRotation()), vehicleToGoal.getRotation());

        AimingParameters params = new AimingParameters(vehicleToGoal, kDefaultFieldRelativeGoalLocation,
                kDefaultFieldRelativeGoalLocation.getRotation(), 0, 0, -1);

        return Optional.of(params);
    }

    public synchronized Optional<AimingParameters> getAimingParameters(int prev_track_id, double max_track_age) {
        GoalTracker tracker = goal_tracker_;
        List<GoalTracker.TrackReport> reports = tracker.getTracks();

        if (reports.isEmpty()) {
            return Optional.empty();
        }

        double timestamp = Timer.getFPGATimestamp();

        // Find the best track.
        TrackReportComparator comparator = new TrackReportComparator(
                Constants.VisionConstants.kTrackStabilityWeight,
                Constants.VisionConstants.kTrackAgeWeight,
                Constants.VisionConstants.kTrackSwitchingWeight,
                prev_track_id,
                timestamp);
                
        reports.sort(comparator);

        GoalTracker.TrackReport report = null;
        for (GoalTracker.TrackReport track : reports) {
            if (track.latest_timestamp > timestamp - max_track_age) {
                report = track;
                break;
            }
        }
        if (report == null) {
            return Optional.empty();
        }

        Pose2d vehicleToGoal = getFieldToVehicle(timestamp).inverse().transformBy(report.field_to_target);
        vehicleToGoal = new Pose2d(vehicleToGoal.getTranslation().rotateBy(getFieldToVehicle(timestamp).getRotation()), vehicleToGoal.getRotation());

        AimingParameters params = new AimingParameters(vehicleToGoal, report.field_to_target,
                report.field_to_target.getRotation(), report.latest_timestamp, report.stability, report.id);
        SmartDashboard.putString("Field to Target", report.field_to_target.toString());
        return Optional.of(params);
    }

    public Pose2d getRobot() {
        return new Pose2d();
    }

    public synchronized Pose2d getVisionTargetToGoalOffset() {
        return Pose2d.fromTranslation(new Translation2d(0, 0));
    }

    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());
        SmartDashboard.putString("Robot Field to Vehicle", getLatestFieldToVehicle().getValue().toString());
        SmartDashboard.putNumber("Robot Field To Vehicle X", getLatestFieldToVehicle().getValue().getTranslation().x());
        SmartDashboard.putNumber("Robot Field To Vehicle Y", getLatestFieldToVehicle().getValue().getTranslation().y());
        SmartDashboard.putNumber("Robot Field To Vehicle Theta", getLatestFieldToVehicle().getValue().getRotation().getDegrees());
        SmartDashboard.putString("Smoothed Predicted Velocity", getSmoothedPredictedVelocity().toString());

        Optional<AimingParameters> params = getAimingParameters(-1, Constants.VisionConstants.kMaxGoalTrackAge);
        SmartDashboard.putBoolean("Has Aiming Parameters", params.isPresent());
        if (params.isPresent()) {
            SmartDashboard.putString("Vehicle to Target", params.get().getVehicleToGoal().toString());
            SmartDashboard.putNumber("Vehicle to Target Angle", params.get().getVehicleToGoalRotation().getDegrees());
        }
    }
}
