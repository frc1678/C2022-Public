package com.team1678.frc2022.shuffleboard;

import java.util.List;

import com.team1678.frc2022.shuffleboard.tabs.ClimberTab;
import com.team1678.frc2022.shuffleboard.tabs.ColorSensorTab;
import com.team1678.frc2022.shuffleboard.tabs.IndexerTab;
import com.team1678.frc2022.shuffleboard.tabs.IntakeTab;
import com.team1678.frc2022.shuffleboard.tabs.LedTab;
import com.team1678.frc2022.shuffleboard.tabs.ManualShooterTab;
import com.team1678.frc2022.shuffleboard.tabs.OperatorTab;
import com.team1678.frc2022.shuffleboard.tabs.ShooterTab;
import com.team1678.frc2022.shuffleboard.tabs.SuperstructureTab;
import com.team1678.frc2022.shuffleboard.tabs.SwerveTab;
import com.team1678.frc2022.shuffleboard.tabs.VisionTab;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleBoardInteractions {

    // Trims unneccesary tabs when in competition
    public final boolean mDebug = true;

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance; 

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    private List<ShuffleboardTabBase> mTabs;

    private OperatorTab mOperatorTab = new OperatorTab();
    private FieldView mFieldView = new FieldView();

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        mTabs = List.of(
            mOperatorTab,
            new ColorSensorTab(),
            new SuperstructureTab(),
            new SwerveTab(),
            new VisionTab()
        );

        if (mDebug) {
            mTabs.addAll(List.of(
                new ClimberTab(),
                new IndexerTab(),
                new IntakeTab(),
                new LedTab(),
                new ManualShooterTab(),
                new ShooterTab()
            ));
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
        mFieldView.update();
    }

    public ShuffleboardTab getOperatorTab() {
        return mOperatorTab.getTab();
    }
}
 