package com.team1678.frc2022.regressions;

public class EpsilonRegression {
    public static double[][] kHoodManualAngle = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> hood angle (in degrees)
        { 2.0, 20.0 },
        { 2.5, 25.0 },
        { 3.0, 27.0 },
        { 3.5, 29.0 },
        { 4.0, 32.0 },
        { 4.5, 34.0 },
        { 5.0, 34.0 },
        { 5.5, 35.0 },
        { 6.0, 35.0 },
    };

    public static double[][] kFlywheelManualRPM = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rpm)
        { 2.0, 2200 },
        { 2.5, 2300 },
        { 3.0, 2300 },
        { 3.5, 2350 },
        { 4.0, 2450 },
        { 4.5, 2550 },
        { 5.0, 2650 },
        { 5.5, 2750 },
        { 6.0, 2850 },

    };
}
