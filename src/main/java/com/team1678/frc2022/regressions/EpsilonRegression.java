package com.team1678.frc2022.regressions;

public class EpsilonRegression {
    public static double[][] kHoodManualAngle = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> hood angle (in degrees)
        { 1.0, 11.0 },
        { 1.25, 12.0 },
        { 1.50, 16.0 },
        { 1.75, 18.0 },
        { 2.0, 20.0 },
        { 2.25, 24.0 },
        { 2.5, 25.0 },
        { 2.75, 26.0 },
        { 3.0, 27.0 },
        { 3.25, 28.0 },
        { 3.5, 29.0 },
        { 3.75, 31.0 },
        { 4.0, 32.0 },
        { 4.25, 33.0 },
        { 4.5, 34.0 },
        { 4.75, 34.0 },
        { 5.0, 34.0 },
        { 5.5, 35.0 },
        { 6.0, 35.0 },
        { 6.5, 35.0 },
        { 7.0, 35.0 }
    };

    public static double[][] kFlywheelManualRPM = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rpm)
        { 1.0, 2050 },
        { 1.25, 2050 },
        { 1.50, 2200 },
        { 1.75, 2200 },
        { 2.0, 2200 },
        { 2.25, 2200 },
        { 2.5, 2300 },
        { 2.75, 2300 },
        { 3.0, 2350 },
        { 3.25, 2400 },
        { 3.5, 2420 },
        { 3.75, 2450 },
        { 4.0, 2500 },
        { 4.25, 2520 },
        { 4.5, 2600 },
        { 4.75, 2700 },
        { 5.0, 2710 },
        { 5.5, 2780 },
        { 6.0, 2900 },
        { 6.5, 3000 },
        { 7.0, 3150 }

    };
}
