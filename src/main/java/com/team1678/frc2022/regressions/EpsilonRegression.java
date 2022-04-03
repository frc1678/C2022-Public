package com.team1678.frc2022.regressions;

public class EpsilonRegression {
    public static double[][] kHoodManualAngle = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> hood angle (in degrees)
        { 2.0, 22.0 },
        { 3.0, 30.0 },
 
        { 4.0, 35.0 },
    };

    public static double[][] kFlywheelManualRPM = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rpm)
        { 2.0, 2200 },

        { 3.0, 2350 },

        { 4.0, 2700 },
    };
}
