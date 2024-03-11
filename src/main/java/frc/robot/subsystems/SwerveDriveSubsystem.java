// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableRegistry;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
//import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pigeon2Handler;
import frc.robot.util.Constants;


public class SwerveDriveSubsystem extends SubsystemBase {

    public final double W = Constants.DRIVE_TRAIN_WIDTH / 2;
    public final double L = Constants.DRIVE_TRAIN_LENGTH / 2;

    public SwerveWheelModuleSubsystem backRight;
    public SwerveWheelModuleSubsystem backLeft;
    public SwerveWheelModuleSubsystem frontRight;
    public SwerveWheelModuleSubsystem frontLeft;
    public SwerveDriveKinematics kinematics;
    public static ChassisSpeeds speeds;
    public PIDController angleController;
    public Pigeon2Handler pigeon;
    public SwerveDriveOdometry odometry;
    
    public SwerveDriveSubsystem(Pigeon2Handler pigeon) {
        //calibrateWidget = Shuffleboard.getTab("Preferences").addPersistent("Calibrate?", false)
                //.withWidget(BuiltInWidgets.kToggleButton);
        // invertWidget = Shuffleboard.getTab("Preferences").addPersistent("Invert?", false)
        //         .withWidget(BuiltInWidgets.kToggleButton);

        backRight = new SwerveWheelModuleSubsystem(7, 8, 23, "BR", Constants.bROffset);// These are the motors and encoder
                                                                // CAN IDs for swerve drive
        backLeft = new SwerveWheelModuleSubsystem(1, 2, 20, "BL", Constants.bLOffset);
        frontRight = new SwerveWheelModuleSubsystem(5, 6, 22, "FR", Constants.fROffset);
        frontLeft = new SwerveWheelModuleSubsystem(3, 4, 21, "FL", Constants.fLOffset);// The order is angle, speed,
                                                                                   // encoder, calibrateWidget
        SendableRegistry.addChild(this, backRight);
        SendableRegistry.addChild(this, backLeft);
        SendableRegistry.addChild(this, frontRight);
        SendableRegistry.addChild(this, frontLeft);

        SendableRegistry.addLW(this, "Swerve Drive Subsystem");

        // angleController = new PIDController(.3, 0, 0); //this pid controller is for controlling total robot angle
        //angleController.enableContinuousInput(0, Math.PI * 2); 
        //angleController.setSetpoint(0); 
        
        Translation2d backRightLocation = new Translation2d(-L, -W);
        Translation2d backLeftLocation = new Translation2d(-L, W);
        Translation2d frontRightLocation = new Translation2d(L, -W); 
        Translation2d frontLeftLocation = new Translation2d(L, W);

        kinematics = new SwerveDriveKinematics(backRightLocation, backLeftLocation, frontRightLocation, frontLeftLocation);
        speeds = new ChassisSpeeds(0, 0, 0);
        this.pigeon = pigeon;

        odometry = new SwerveDriveOdometry
                    (kinematics, 

                    pigeon.getAngleRad(), 

                    new SwerveModulePosition[] {
                        backRight.getSwerveModulePosition(), 
                        backLeft.getSwerveModulePosition(),
                        frontRight.getSwerveModulePosition(),
                        frontLeft.getSwerveModulePosition()});
                        
        
    }
    //This drive sets new speeds from the controller
    //This class will periodically calculate new SwerveDriveModule positions based on these ChassisSpeeds
    //Then, each module will be called to "drive" to these positions
    public void drive(ChassisSpeeds chassisSpeeds) {
        // implementation from Pride of the North
        speeds = chassisSpeeds;
    }

    public void setSwerveModuleStates(SwerveModuleState[] states)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 5); 
        //SwerveDriveKinematics.desaturateWheelSpeeds(states, speeds, 5, Constants.kMaxVelocity, Constants.kMaxAngularVelocity);
        // Front left module state
        SwerveModuleState backRightState = states[0];

        // Front right module state
        SwerveModuleState backLeftState = states[1];

        // Back left module state
        SwerveModuleState frontRightState = states[2];

        // Back right module state
        SwerveModuleState frontLeftState = states[3];
        
        backLeft.drive(backLeftState.speedMetersPerSecond, backLeftState.angle.getDegrees()); //5.5 m/s is maximum zero load velocity
        backRight.drive(backRightState.speedMetersPerSecond, backRightState.angle.getDegrees()); // removed negative sign
        frontLeft.drive(frontLeftState.speedMetersPerSecond, frontLeftState.angle.getDegrees());
        frontRight.drive(frontRightState.speedMetersPerSecond, frontRightState.angle.getDegrees()); // removed negative sign 12/29/23
        
    }

    // @Override
    public void periodic() {
        // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        setSwerveModuleStates(moduleStates);

        odometry.update(pigeon.getAngleRad(), 
                        new SwerveModulePosition[] {
                            backRight.getSwerveModulePosition(), 
                            backLeft.getSwerveModulePosition(),
                            frontRight.getSwerveModulePosition(),
                            frontLeft.getSwerveModulePosition()});

        SmartDashboard.putNumber("PoseX", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Pose Theta", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Readings", pigeon.getAngleDeg().getDegrees());

    
    }

    public void stop() {
        backRight.stop();
        backLeft.stop();
        frontRight.stop();
        frontLeft.stop();
    }

    public void coast(){
        backRight.coast();
        backLeft.coast();
        frontRight.coast();
        frontLeft.coast();
    }

    public void brake(){
        speeds = new ChassisSpeeds(0,0,0.0001);
        backRight.brake();
        backLeft.brake();
        frontRight.brake();
        frontLeft.brake();
        
    }

    public void currentLimit(){
        backRight.swerveCurrentLimit();
        backLeft.swerveCurrentLimit();
        frontRight.swerveCurrentLimit();
        frontLeft.swerveCurrentLimit();
    }

    public Rotation2d getRobotAngle()
    {
        return pigeon.getAngleDeg();
    }

    public void resetPose()
    {
        odometry.resetPosition(pigeon.getAngleRad(), 
                                new SwerveModulePosition[] {
                                    backRight.getSwerveModulePosition(), 
                                    backLeft.getSwerveModulePosition(),
                                    frontRight.getSwerveModulePosition(),
                                    frontLeft.getSwerveModulePosition()},
                                new Pose2d(0, 0, pigeon.getAngleRad()));
    }

    public void setPose(double x, double y, double heading)
    {
        odometry.resetPosition(pigeon.getAngleRad(), 
                                new SwerveModulePosition[] {
                                    backRight.getSwerveModulePosition(), 
                                    backLeft.getSwerveModulePosition(),
                                    frontRight.getSwerveModulePosition(),
                                    frontLeft.getSwerveModulePosition()},
                                new Pose2d(x, y, new Rotation2d(heading)));
    }


    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getRobotPose()
    {
        return odometry.getPoseMeters();
    }
    public Supplier<Pose2d> getRobotPoseSupplier()
    {
        return odometry::getPoseMeters;
    }
}

