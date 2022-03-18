package com.team1678.frc2022.regressions;

import com.team254.lib.util.PolynomialRegression;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.InterpolatingDouble;


public class ShooterRegression {
    public static final double kHoodPaddingDegrees = 2;
    public static final double kShooterPaddingVelocity = 100;


    public static final double[] kPadding = {
            kShooterPaddingVelocity, kHoodPaddingDegrees};

    //hood
    public static double kDefaultHoodAngle = Math.toRadians(0);
    public static boolean kUseHoodAutoAimPolynomial = false;

    public static boolean kUseSmartdashboard = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kHoodAutoAimPolynomial;

    public static double[][] kHoodManualAngle = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> hood angle (in degrees)
        { 1.00, 14.0 },
        { 1.25, 17.0 },
        { 1.50, 20.0 },
        { 1.81, 23.0 },
        { 2.04, 24.0 },
        { 2.30, 25.0 },
        { 2.55, 26.0 },
        { 2.81, 27.0 },
        { 3.04, 28.0 },
        { 3.29, 29.0 },
        { 3.55, 31.0 },
        { 3.78, 32.0 },
        { 4.06, 33.0 },
        { 4.29, 34.0 },
        { 4.53, 35.0 },
        { 4.78, 35.0 },
        { 5.29, 35.0 },
        { 6.78, 35.0 },
    };

    static {
        //iterate through the array and place each point into the interpolating tree
        for (double[] pair : kHoodManualAngle) {
            kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        
        kHoodAutoAimPolynomial = new PolynomialRegression(kHoodManualAngle, 1);
    }
    
    //shooter
    public static double kDefaultShootingRPM = 2950.0;
    public static boolean kUseFlywheelAutoAimPolynomial = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kFlywheelAutoAimPolynomial;

    public static double[][] kFlywheelManualRPM = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rpm)
        { 1.00, 2150 },
        { 1.25, 2150 },
        { 1.50, 2150 },
        { 1.81, 2150 },
        { 2.04, 2200 },
        { 2.30, 2300 },
        { 2.55, 2300 },
        { 2.81, 2350 },
        { 3.04, 2400 },
        { 3.29, 2420 },
        { 3.55, 2450 },
        { 3.78, 2500 },
        { 4.06, 2520 },
        { 4.29, 2600 },
        { 4.53, 2700 },
        { 4.78, 2730 },
        { 5.29, 2800 },
        { 6.78, 2900 }, //  3200

    };

    static {
        for (double[] pair : kFlywheelManualRPM) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelManualRPM, 2);
    }

}
