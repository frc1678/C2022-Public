package com.team254.lib.util;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import java.util.ArrayList;

/**
 * Helper class for storing and calculating a moving average of the pose2d class
 */
public class MovingAveragePose2d {
    ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
    private int maxSize;

    public MovingAveragePose2d(int maxSize) {
        this.maxSize = maxSize;
    }

    public synchronized void add(Pose2d pose) {
        poses.add(pose);
        if (poses.size() > maxSize) {
            poses.remove(0);
        }
    }

    public synchronized Pose2d getAverage() {
        double x = 0.0, y = 0.0, t = 0.0;

        for (Pose2d pose : poses) {
            x += pose.getTranslation().x();
            y += pose.getTranslation().y();
            t += pose.getRotation().getDegrees();
        }

        double size = getSize();
        return new Pose2d(x / size, y / size, new Rotation2d(t / size));
    }

    public int getSize() {
        return poses.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        poses.clear();
    }

}