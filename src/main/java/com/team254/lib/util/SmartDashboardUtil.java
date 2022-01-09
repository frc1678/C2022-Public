package com.team254.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Small class filled with static util methods for the SmartDashboard.
 */
public class SmartDashboardUtil {

    public static void deletePersistentKeys() {
        for (String key : SmartDashboard.getKeys()) {
            if (SmartDashboard.isPersistent(key)) {
                SmartDashboard.delete(key);
            }
        }
    }

    public static void deleteAllKeys() {
        for (String key : SmartDashboard.getKeys()) {
            SmartDashboard.delete(key);
        }
    }
}
