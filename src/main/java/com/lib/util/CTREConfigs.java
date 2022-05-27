package com.lib.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.team1678.frc2022.Constants;

public final class CTREConfigs {
    public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveConstants.driveEnableCurrentLimit, 
            Constants.SwerveConstants.driveContinuousCurrentLimit, 
            Constants.SwerveConstants.drivePeakCurrentLimit, 
            Constants.SwerveConstants.drivePeakCurrentDuration);

        config.slot0.kP = Constants.SwerveConstants.driveKP;
        config.slot0.kI = Constants.SwerveConstants.driveKI;
        config.slot0.kD = Constants.SwerveConstants.driveKD;
        config.slot0.kF = Constants.SwerveConstants.driveKF;        
        config.supplyCurrLimit = driveSupplyLimit;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        config.openloopRamp = Constants.SwerveConstants.openLoopRamp;
        config.closedloopRamp = Constants.SwerveConstants.closedLoopRamp;
        return config;
    }

    public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveConstants.angleEnableCurrentLimit, 
            Constants.SwerveConstants.angleContinuousCurrentLimit, 
            Constants.SwerveConstants.anglePeakCurrentLimit, 
            Constants.SwerveConstants.anglePeakCurrentDuration);

        angleConfig.slot0.kP = Constants.SwerveConstants.angleKP;
        angleConfig.slot0.kI = Constants.SwerveConstants.angleKI;
        angleConfig.slot0.kD = Constants.SwerveConstants.angleKD;
        angleConfig.slot0.kF = Constants.SwerveConstants.angleKF;
        angleConfig.supplyCurrLimit = angleSupplyLimit;
        angleConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        return angleConfig;
    }

    public static CANCoderConfiguration swerveCancoderConfig() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = Constants.SwerveConstants.canCoderInvert;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        return config;
    }
}
