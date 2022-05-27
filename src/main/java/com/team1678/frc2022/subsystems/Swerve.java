package com.team1678.frc2022.subsystems;

import java.util.ArrayList;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.RobotState;
import com.team1678.frc2022.drivers.Pigeon;
import com.team1678.frc2022.drivers.SwerveModule;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.util.TimeDelayedBoolean;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem {

    private static Swerve mInstance;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    // status variable for being enabled
    public boolean mIsEnabled = false;

    // limelight instance for raw aiming
    Limelight mLimelight = Limelight.getInstance();

    // logger
    LogStorage<PeriodicIO> mStorage = null;

    // wants vision aim during auto
    public boolean mWantsAutoVisionAim = false;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;

    public Pigeon mPigeon = Pigeon.getInstance();

    // chassis velocity status
    ChassisSpeeds chassisVelocity = new ChassisSpeeds();

    public boolean isSnapping;
    private double mLimelightVisionAlignGoal;
    private double mGoalTrackVisionAlignGoal;
    private double mVisionAlignAdjustment;

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
        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, mPigeon.getYaw().getWPIRotation2d());
        
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
                mIsEnabled = true;
            }

            @Override
            public void onLoop(double timestamp) {
                mIsEnabled = false;
                chooseVisionAlignGoal();
                updateSwerveOdometry();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }
    
    public void setWantAutoVisionAim(boolean aim) {
        mWantsAutoVisionAim = aim;
    } 

    public boolean getWantAutoVisionAim() {
        return mWantsAutoVisionAim;
    }

    public void visionAlignDrive(Translation2d translation2d, boolean fieldRelative) {
        drive(translation2d, mVisionAlignAdjustment, fieldRelative, false);
    }

    public void angleAlignDrive(Translation2d translation2d, double targetHeading, boolean fieldRelative) {
        snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(targetHeading), 0.0));
        double angleAdjustment = snapPIDController.calculate(mPigeon.getYaw().getRadians());
        drive(translation2d, angleAdjustment, fieldRelative, false);
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
                                        mPigeon.getYaw().getWPIRotation2d()
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

    public void acceptLatestGoalTrackVisionAlignGoal(double vision_goal) {
        mGoalTrackVisionAlignGoal = vision_goal; 
    }

    public void chooseVisionAlignGoal() {
        double currentAngle = mPigeon.getYaw().getRadians();
        if (mLimelight.hasTarget()) {
            double targetOffset = Math.toRadians(mLimelight.getOffset()[0]);
            mLimelightVisionAlignGoal = MathUtil.inputModulus(currentAngle - targetOffset, 0.0, 2 * Math.PI);
            visionPIDController.setSetpoint(mLimelightVisionAlignGoal);
        } else {
            visionPIDController.setSetpoint(mGoalTrackVisionAlignGoal);
        }

        mVisionAlignAdjustment = visionPIDController.calculate(currentAngle);
    }

    public double calculateSnapValue() {
        return snapPIDController.calculate(mPigeon.getYaw().getRadians());
    }

    public void startSnap(double snapAngle) {
        snapPIDController.reset(mPigeon.getYaw().getRadians());
        snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
        isSnapping = true;
    }
    
    TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    private boolean snapComplete() {
        double error = snapPIDController.getGoal().position - mPigeon.getYaw().getRadians();
        return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.kEpsilon), Constants.SnapConstants.kTimeout);
    }

    public void maybeStopSnap(boolean force){
        if (!isSnapping) {
            return;
        } 
        if (force || snapComplete()) {
            isSnapping = false;
            snapPIDController.reset(mPigeon.getYaw().getRadians());
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            SmartDashboard.putNumber("mod " + mod.moduleNumber +  " desired speed", desiredStates[mod.moduleNumber].speedMetersPerSecond);
            SmartDashboard.putNumber("mod " + mod.moduleNumber +  " desired angle", MathUtil.inputModulus(desiredStates[mod.moduleNumber].angle.getDegrees(), 0, 180));
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, pose.getRotation());
        zeroGyro(pose.getRotation().getDegrees());

        // reset field to vehicle
        RobotState.getInstance().reset(new com.team254.lib.geometry.Pose2d(pose));
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
            SmartDashboard.putNumber("mod " + mod.moduleNumber + " current speed", states[mod.moduleNumber].speedMetersPerSecond);
            SmartDashboard.putNumber("mod " + mod.moduleNumber + " current angle", MathUtil.inputModulus(states[mod.moduleNumber].angle.getDegrees(), 0, 180));
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
        zeroGyro(0.0);
    }
    
    public void zeroGyro(){
        zeroGyro(0.0);
    }

    public void zeroGyro(double reset){
        mPigeon.setYaw(reset);
        visionPIDController.reset();
    }

    public void updateSwerveOdometry(){
        swerveOdometry.update(mPigeon.getYaw().getWPIRotation2d(), getStates());

        chassisVelocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(
                    mInstance.mSwerveMods[0].getState(),
                    mInstance.mSwerveMods[1].getState(),
                    mInstance.mSwerveMods[2].getState(),
                    mInstance.mSwerveMods[3].getState()
            );
    }

    @Override
    public void stop() {
        mIsEnabled = false;
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
        mPeriodicIO.pigeon_heading = mPigeon.getYaw().getDegrees();
        mPeriodicIO.robot_pitch = mPigeon.getUnadjustedPitch().getDegrees();
        mPeriodicIO.robot_roll = mPigeon.getRoll().getDegrees();
        mPeriodicIO.snap_target = Math.toDegrees(snapPIDController.getGoal().position);
        mPeriodicIO.vision_align_target_angle = Math.toDegrees(mLimelightVisionAlignGoal);
        mPeriodicIO.swerve_heading = MathUtil.inputModulus(mPigeon.getYaw().getDegrees(), 0, 360);

        SendLog();
    }

    public static class PeriodicIO {
        // inputs
        public double odometry_pose_x;
        public double odometry_pose_y;
        public double odometry_pose_rot;

        public double pigeon_heading;
        public double robot_pitch;
        public double robot_roll;
        public double vision_align_target_angle;
        public double swerve_heading;

        public double angular_velocity;
        public double goal_velocity;

        public double profile_position;

        // outputs
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
        headers.add("timestamp");
        headers.add("is_enabled");
        headers.add("odometry_pose_x");
        headers.add("odometry_pose_y");
        headers.add("odometry_pose_rot");
        headers.add("pigeon_heading");
        headers.add("robot_pitch");
        headers.add("robot_roll");
        headers.add("snap_target");
        headers.add("vision_align_target_angle");
        headers.add("swerve_heading");
        for (SwerveModule module : this.mSwerveMods) {
            headers.add(module.moduleNumber + "_angle");
            headers.add(module.moduleNumber + "_desired_angle");
            headers.add(module.moduleNumber + "_velocity");
            headers.add(module.moduleNumber + "_cancoder");
        }

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());
        items.add(mIsEnabled ? 1.0 : 0.0);
        items.add(mPeriodicIO.odometry_pose_x);
        items.add(mPeriodicIO.odometry_pose_y);
        items.add(mPeriodicIO.odometry_pose_rot);
        items.add(mPeriodicIO.pigeon_heading);
        items.add(mPeriodicIO.robot_pitch);
        items.add(mPeriodicIO.robot_roll);
        items.add(mPeriodicIO.snap_target);
        items.add(mPeriodicIO.vision_align_target_angle);
        items.add(mPeriodicIO.swerve_heading);
        for (SwerveModule module : this.mSwerveMods) {
            items.add(module.getState().angle.getDegrees());
            items.add(module.getTargetAngle());
            items.add(module.getState().speedMetersPerSecond);
            items.add(MathUtil.inputModulus(module.getCanCoder().getDegrees() - module.angleOffset, 0, 360));
        }

        // send data to logging storage
        mStorage.addData(items);
    }

}

