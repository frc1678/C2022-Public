package com.team254.lib.vision;

import com.team1678.frc2022.Constants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Map;
import java.util.TreeMap;


/**
 * A class that is used to keep track of all goals detected by the vision system. As goals are detected/not detected
 * anymore by the vision system, function calls will be made to create, destroy, or update a goal track.
 * <p>
 * This helps in the goal ranking process that determines which goal to fire into, and helps to smooth measurements of
 * the goal's location over time.
 *
 * @see GoalTracker
 */
public class GoalTrack {
    TreeMap<Double, Pose2d> mObservedPositions = new TreeMap<>();

    Pose2d mSmoothedPosition = null;
    int mId;

    double mx = 0;
    double my = 0;
    double ms = 0;  // sin of angle
    double mc = 0;  // cos of angle

    private GoalTrack() {}

   public synchronized void addPoint(double timestamp, Pose2d new_observation) {
       mObservedPositions.put(timestamp, new_observation);
       mx += new_observation.getTranslation().x();
       my += new_observation.getTranslation().y();
       ms += new_observation.getRotation().sin();
       mc += new_observation.getRotation().cos();
   }

   public synchronized void removePoint(double timestamp, Pose2d old_observation) {
       mx -= old_observation.getTranslation().x();
       my -= old_observation.getTranslation().y();
       ms -= old_observation.getRotation().sin();
       mc -= old_observation.getRotation().cos();
       mObservedPositions.remove(timestamp, old_observation);
   }

    /**
     * Makes a new track based on the timestamp and the goal's coordinates (from vision)
     */
    public static synchronized GoalTrack makeNewTrack(double timestamp, Pose2d first_observation, int id) {
        GoalTrack rv = new GoalTrack();
        rv.addPoint(timestamp, first_observation);
        rv.mSmoothedPosition = first_observation;
        rv.mId = id;
        return rv;
    }

    public synchronized void emptyUpdate() {
        pruneByTime();
    }

    /**
     * Attempts to update the track with a new observation.
     *
     * @return True if the track was updated
     */
    public synchronized boolean tryUpdate(double timestamp, Pose2d new_observation) {
        if (!isAlive()) {
            return false;
        }
        double distance = mSmoothedPosition.inverse().transformBy(new_observation).getTranslation().norm();
        if (distance < Constants.VisionConstants.kMaxTrackerDistance) {
            addPoint(timestamp, new_observation);
            pruneByTime();
            return true;
        } else {
            emptyUpdate();
            return false;
        }
    }

    public synchronized boolean isAlive() {
        return mObservedPositions.size() > 0;
    }

    /**
     * Removes the track if it is older than the set "age" described in the Constants file.
     *
     * @see Constants
     */
    synchronized void pruneByTime() {
        double delete_before = Timer.getFPGATimestamp() - Constants.VisionConstants.kMaxGoalTrackAge;
        /* remove all old points one at a time */
        for (Map.Entry<Double, Pose2d> entry : mObservedPositions.entrySet()) {
            if (entry.getKey() < delete_before) {
                removePoint(entry.getKey(), entry.getValue());
            }
        }
        //mObservedPositions.entrySet().removeIf(entry -> entry.getKey() < delete_before);
        if (mObservedPositions.isEmpty()) {
            mSmoothedPosition = null;
        } else {
            smooth();
        }
    }

    /**
     * Averages out the observed positions based on an set of observed positions
     */
    synchronized void smooth() {
        if (isAlive()) {
            int num_samples = mObservedPositions.size();

            if (num_samples == 0) {
                // Handle the case that all samples are older than kMaxGoalTrackSmoothingTime.
                mSmoothedPosition = mObservedPositions.lastEntry().getValue();
            } else {
                double x = mx / num_samples;
                double y = my / num_samples;
                double s = ms / num_samples;  // sin of angle
                double c = mc / num_samples;  // cos of angle
                mSmoothedPosition = new Pose2d(x, y, new Rotation2d(c, s, true));
            }
        }
    }

    public synchronized Pose2d getSmoothedPosition() {
        return mSmoothedPosition;
    }

    public synchronized Pose2d getLatestPosition() {
        return mObservedPositions.lastEntry().getValue();
    }

    public synchronized double getLatestTimestamp() {
        return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
    }

    public synchronized double getStability() {
        return Math.min(1.0, mObservedPositions.size() / (Constants.VisionConstants.kCameraFrameRate * Constants.VisionConstants.kMaxGoalTrackAge));
    }

    public synchronized int getId() {
        return mId;
    }
}
