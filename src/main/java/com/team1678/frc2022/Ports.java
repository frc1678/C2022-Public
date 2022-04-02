package com.team1678.frc2022;

public class Ports {

    /*** SWERVE MODULE PORTS ***/

    /*  
    Swerve Modules go:
        1 2
        3 4
    */

    public static final int FL_DRIVE = 0;
    public static final int FL_ROTATION = 1;
    public static final int FL_CANCODER = 0; 

    public static final int FR_DRIVE = 2; 
    public static final int FR_ROTATION = 3;
    public static final int FR_CANCODER = 1; 

    public static final int BL_DRIVE = 4;
    public static final int BL_ROTATION = 5;
    public static final int BL_CANCODER = 2; 

    public static final int BR_DRIVE = 6;
    public static final int BR_ROTATION = 7;
    public static final int BR_CANCODER = 3; 

    // Pigeon
    public static final int PIGEON = 20;

    /*** SUBSYSTEM IDS ***/
    public static final int INTAKE_ROLLER_ID = 8;
    public static final int INTAKE_DEPLOY_ID = 9;

    public static final int SINGULATOR_ID = 10;
    public static final int TUNNEL_ID = 11;
    public static final int TRIGGER_ID = 12;

    public static int getForwardBeamBreakPort() {
        return Constants.isComp ? 4 : 7;
    }

    public static int getBottomBeamBreakPort() {
        return Constants.isComp ? 3 : 1;
    }

    public static int getTopBeamBreakPort() {
        return Constants.isComp ? 2 : 4;
    }

    public static final int FLYWHEEL_MASTER_ID = 13;
    public static final int FLYWHEEL_SLAVE_ID = 14;
    public static final int HOOD_ID = 15;
    
    public static final int CLIMBER_LEFT_ID = 16;
    public static final int CLIMBER_RIGHT_ID = 17;

    public static final int EJECTOR_ID = 18;

    // Infrastucture
    public static final int PCM = 21;

    // Candle
    public static final int CANDLE = 22;


}
