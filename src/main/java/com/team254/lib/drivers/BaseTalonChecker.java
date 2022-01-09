package com.team254.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.team1678.frc2022.subsystems.Subsystem;

import java.util.ArrayList;

public class BaseTalonChecker extends MotorChecker<BaseTalon> {
    private static class StoredBaseTalonConfiguration {
        public ControlMode mMode;
        public double mSetValue;
    }

    protected ArrayList<StoredBaseTalonConfiguration> mStoredConfigurations = new ArrayList<>();

    public static boolean checkMotors(Subsystem subsystem,
                                      ArrayList<MotorConfig<BaseTalon>> motorsToCheck,
                                      CheckerConfig checkerConfig) {
        BaseTalonChecker checker = new BaseTalonChecker();
        return checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfiguration() {
        // record previous configuration for all talons
        for (MotorConfig<BaseTalon> config : mMotorsToCheck) {
            // LazyBaseTalon talon = (LazyBaseTalon) config.mMotor;

            StoredBaseTalonConfiguration configuration = new StoredBaseTalonConfiguration();
            configuration.mMode = ControlMode.PercentOutput; //talon.getControlMode();
            configuration.mSetValue = 0.0; //talon.getLastSet();

            mStoredConfigurations.add(configuration);
        }
    }

    @Override
    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.size(); ++i) {
            mMotorsToCheck.get(i).mMotor.set(mStoredConfigurations.get(i).mMode,
                    mStoredConfigurations.get(i).mSetValue);
        }
    }

    @Override
    protected void setMotorOutput(BaseTalon motor, double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMotorCurrent(BaseTalon motor) {
        return motor.getStatorCurrent();
    }
}
