package com.team1678.frc2022.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.RobotState;
import com.team1678.frc2022.SwerveModule;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem {

    private static Swerve mInstance;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    // required instance for vision align
    public Limelight mLimelight = Limelight.getInstance();

    // track id
    private int mTrackId = -1;

    // wants vision aim during auto
    public boolean mWantsAutoVisionAim = false;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public boolean isSnapping;
    private double mVisionAlignAdjustment;
    private double mVisionAlignGoal;
    private double mCorrectedDistanceToTarget;

    public ProfiledPIDController snapPIDController;
    public PIDController visionPIDController;

    
    // Private boolean to lock Swerve wheels
    private boolean mLocked = false;
    // Getter
    public boolean getLocked() {
        return mLocked;
    }
    // Setter
    public void setLocked(boolean lock) {
        mLocked = lock;
    }

    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    public Swerve() {
        gyro = new Pigeon2(Ports.PIGEON, "canivore1");
        gyro.configFactoryDefault();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw());
        
        snapPIDController = new ProfiledPIDController(Constants.SnapConstants.kP,
                                                      Constants.SnapConstants.kI, 
                                                      Constants.SnapConstants.kD,
                                                      Constants.SnapConstants.kThetaControllerConstraints);
        snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

        visionPIDController = new PIDController(Constants.VisionAlignConstants.kP,
                                                        Constants.VisionAlignConstants.kI,
                                                        Constants.VisionAlignConstants.kD);
        visionPIDController.enableContinuousInput(-Math.PI, Math.PI);
        visionPIDController.setTolerance(0.0);

        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.SwerveModuleConstants()),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.SwerveModuleConstants()),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.SwerveModuleConstants()),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.SwerveModuleConstants())
        };
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                updateSwerveOdometry();
                updateVisionAimingSetpoints();
                outputTelemetry();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }
    
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Is Snapping", isSnapping);
        for(SwerveModule mod : mSwerveMods){
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", MathUtil.inputModulus(mod.getCanCoder().getDegrees() - mod.angleOffset, 0, 360));
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putBoolean("Wants Auto Vision Aim", mWantsAutoVisionAim);
        SmartDashboard.putNumber("Vision Align Target Angle", Math.toDegrees(mVisionAlignGoal));
        SmartDashboard.putNumber("Swerve Heading", MathUtil.inputModulus(getYaw().getDegrees(), 0, 360));
    }

    public void setWantAutoVisionAim(boolean aim) {
        mWantsAutoVisionAim = aim;
    } 

    public boolean getWantAutoVisionAim() {
        return mWantsAutoVisionAim;
    }

    public void visionAlignDrive(Translation2d translation2d, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double adjustedRotation;
        /*
        if (mLimelight.hasTarget()) {
            adjustedRotation = mVisionAlignAdjustment;
        } else {
            adjustedRotation = rotation;
        }
        */
        adjustedRotation = mVisionAlignAdjustment;
        drive(translation2d, adjustedRotation, fieldRelative, isOpenLoop);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if (isSnapping) {
            if (Math.abs(rotation) == 0.0) {
                maybeStopSnap(false);
                rotation = calculateSnapValue();
            } else {
                maybeStopSnap(true);
            }
        }
        SwerveModuleState[] swerveModuleStates = null;
        if (mLocked) {
            swerveModuleStates = new SwerveModuleState[]{
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
            };
        } else {
            swerveModuleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), 
                                        translation.getY(), 
                                        rotation, 
                                        getYaw()
                                    )
                                    : new ChassisSpeeds(
                                        translation.getX(), 
                                        translation.getY(), 
                                        rotation)
                                    );
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    
    
    public void updateVisionAimingSetpoints() {
        double currentAngle = getYaw().getRadians();
        double targetOffset = 0.0;

        Optional<AimingParameters> aiming_params_ = RobotState.getInstance().getAimingParameters(mTrackId, Constants.VisionConstants.kMaxGoalTrackAge);
        if (aiming_params_.isPresent()) {
            mTrackId = aiming_params_.get().getTrackId();
            targetOffset = aiming_params_.get().getVehicleToGoalRotation().getRadians();
            mCorrectedDistanceToTarget = aiming_params_.get().getVehicleToGoal()
        }

        mVisionAlignGoal = MathUtil.inputModulus(currentAngle + targetOffset, 0.0, 2 * Math.PI);

        visionPIDController.setSetpoint(mVisionAlignGoal);
        mVisionAlignAdjustment = visionPIDController.calculate(currentAngle);
    }


    public double calculateSnapValue() {
        return snapPIDController.calculate(getYaw().getRadians());
    }

    public void startSnap(double snapAngle) {
        snapPIDController.reset(getYaw().getRadians());
        snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
        isSnapping = true;
    }
    
    TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    private boolean snapComplete() {
        double error = snapPIDController.getGoal().position - getYaw().getRadians();
        return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.snapEpsilon), Constants.SnapConstants.snapTimeout);
        //return Math.abs(error) < Math.toRadians(Constants.SnapConstants.snapEpsilon);
    }

    public void maybeStopSnap(boolean force){
        if (!isSnapping) {
            return;
        } 
        if (force || snapComplete()) {
            isSnapping = false;
            snapPIDController.reset(getYaw().getRadians());
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, pose.getRotation());
        zeroGyro(pose.getRotation().getDegrees());
    }

    public void resetAnglesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void setAnglePIDValues(double kP, double kI, double kD) {
        for (SwerveModule swerveModule : mSwerveMods) {
            swerveModule.updateAnglePID(kP, kI, kD);
        }
    }

    public double[] getAnglePIDValues(int index) {
        return mSwerveMods[index].getAnglePIDValues();
    }

    public void setVisionAlignPIDValues(double kP, double kI, double kD) {
        visionPIDController.setPID(kP, kI, kD);
    }

    public double[] getVisionAlignPIDValues() {
        return  new double[] {visionPIDController.getP(), visionPIDController.getI(), visionPIDController.getD()};
    }

    @Override
    public void zeroSensors(){
        zeroGyro();
    }
    
    public void zeroGyro(){
        zeroGyro(0);
    }

    public void zeroGyro(double reset){
        gyro.setYaw(reset);
        visionPIDController.reset();
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public Rotation2d getPitch() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[1]) : Rotation2d.fromDegrees(ypr[1]);
    }

    public Rotation2d getRoll() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[2]) : Rotation2d.fromDegrees(ypr[2]);
    }

    public void updateSwerveOdometry(){
        swerveOdometry.update(getYaw(), getStates());  
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.odometry_pose_x = swerveOdometry.getPoseMeters().getX();
        mPeriodicIO.odometry_pose_y = swerveOdometry.getPoseMeters().getY();
        mPeriodicIO.odometry_pose_rot = swerveOdometry.getPoseMeters().getRotation().getDegrees();
        mPeriodicIO.pigeon_heading = getYaw().getDegrees();
        mPeriodicIO.robot_pitch = getPitch().getDegrees();
        mPeriodicIO.robot_roll = getRoll().getDegrees();
        mPeriodicIO.snap_target = Math.toDegrees(snapPIDController.getGoal().position);

        SendLog();
    }

    public static class PeriodicIO {

        public double odometry_pose_x;
        public double odometry_pose_y;
        public double odometry_pose_rot;
        public double pigeon_heading;
        public double robot_pitch;
        public double robot_roll;
        public double snap_target;

    }

    //logger
    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "SWERVE_LOGS.csv");
    }
    
    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("odometry_pose_x");
        headers.add("odometry_pose_y");
        headers.add("odometry_pose_rot");
        headers.add("pigeon_heading");
        headers.add("robot_pitch");
        headers.add("robot_roll");
        headers.add("snap_target");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(mPeriodicIO.odometry_pose_x);
        items.add(mPeriodicIO.odometry_pose_y);
        items.add(mPeriodicIO.odometry_pose_rot);
        items.add(mPeriodicIO.pigeon_heading);
        items.add(mPeriodicIO.robot_pitch);
        items.add(mPeriodicIO.robot_roll);
        items.add(mPeriodicIO.snap_target);

        // send data to logging storage
        mStorage.addData(items);
    }

}
