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
        { 1.0, 2150 },
        { 1.25, 2150 },
        { 1.50, 2300 },
        { 1.75, 2300 },
        { 2.0, 2300 },
        { 2.25, 2300 },
        { 2.5, 2400 },
        { 2.75, 2400 },
        { 3.0, 2450 },
        { 3.25, 2500 },
        { 3.5, 2550 },
        { 3.75, 2600 },
        { 4.0, 2650 },
        { 4.25, 2670 },
        { 4.5, 2750 },
        { 4.75, 2850 },
        { 5.0, 2860 },
        { 5.5, 2930 },
        { 6.0, 3050 },
        { 6.5, 3150 },
        { 7.0, 3300 }

    };
}
