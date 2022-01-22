package com.team1678.frc2022;

import com.team1678.lib.drivers.REVColorSensorV3Wrapper;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper.ColorSensorData;

public class ColorSensor extends Thread{

    private REVColorSensorV3Wrapper mColorSensor;
    private ColorSensorData mRawColorData;

    public ColorSensor(REVColorSensorV3Wrapper colorSensor) {
        mColorSensor = colorSensor;
    }

    public void run() {
        try {
            System.out.println("Running Color Sensor Thread");
            mRawColorData = mColorSensor.getLatestReading();
        } catch (Exception e) {
            System.out.println("Exception is caught");
        }
    }

    public ColorSensorData getColorSensorData() {
        return mRawColorData;
    }

}
