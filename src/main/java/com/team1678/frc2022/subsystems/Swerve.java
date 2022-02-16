package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.SwerveModule;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem {

    private static Swerve mInstance;

    // required instance for vision align
    public Limelight mLimelight = Limelight.getInstance();

    // wants vision aim during auto
    public boolean mWantsAutoVisionAim = false;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public boolean isSnapping;
    public ProfiledPIDController snapPIDController;

    public ProfiledPIDController visionPIDController;

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
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw());
        
        snapPIDController = new ProfiledPIDController(Constants.SnapConstants.kP,
                                                      Constants.SnapConstants.kI, 
                                                      Constants.SnapConstants.kD,
                                                      Constants.SnapConstants.kThetaControllerConstraints);
        snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

        visionPIDController = new ProfiledPIDController(Constants.VisionAlignConstants.kP,
                                                        Constants.VisionAlignConstants.kI,
                                                        Constants.VisionAlignConstants.kD,
                                                        Constants.VisionAlignConstants.kThetaControllerConstraints);
        visionPIDController.enableContinuousInput(-Math.PI, Math.PI);


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
                outputTelemetry();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }
    
    public void outputTelemetry() {
        SmartDashboard.putNumber("Odometry Pose X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Pose Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry Pose Rot", swerveOdometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putBoolean("Is Snapping", isSnapping);
        SmartDashboard.putNumber("Pigeon Heading", getYaw().getDegrees());
        SmartDashboard.putNumber("Snap Target", Math.toDegrees(snapPIDController.getGoal().position));
        for(SwerveModule mod : mSwerveMods){
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", MathUtil.inputModulus(mod.getCanCoder().getDegrees() - mod.angleOffset, 0, 360));
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putBoolean("Wants Auto Vision Aim", mWantsAutoVisionAim);
    }

    public void setWantAutoVisionAim(boolean aim) {
        mWantsAutoVisionAim = aim;
    } 

    public boolean getWantAutoVisionAim() {
        return mWantsAutoVisionAim;
    }

    public void visionAlignDrive(Translation2d translation2d, boolean fieldRelative, boolean isOpenLoop) {
        double rotation = 0.0;

        if (mLimelight.hasTarget() && mLimelight.isOK()) {
            double currentAngle = getPose().getRotation().getRadians();
            double targetOffset = Math.toRadians(mLimelight.getOffset()[0]);
            rotation = visionPIDController.calculate(currentAngle, currentAngle - targetOffset);
        }
        drive(translation2d, rotation, fieldRelative, isOpenLoop);
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
        // zeroGyro(pose.getRotation().getDegrees());
        swerveOdometry.resetPosition(pose, getYaw());
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

    @Override
    public void zeroSensors(){
        zeroGyro();
    }
    
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void zeroGyro(double reset){
        gyro.setYaw(reset);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
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
}
