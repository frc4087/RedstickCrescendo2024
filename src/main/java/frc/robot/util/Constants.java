// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class Constants {
     // Default units: Meters (m/s, m/s/s), Degrees 
    // Assume default unless written otherwise
    // "Forward" (on boot) - +X axis
    // "Left" (on boot) - +Y axis

    public static final double degToRad = Math.PI / 180;
    public static final double radToDeg = 180 / Math.PI;


    public static final double fLOffset = 41.24;//45.35;//44.3;
    public static final double fROffset = 284.85; //286.52;//107.5;//removed 180 
    public static final double bLOffset = 235.72; //238.62;//236.7; //may need to change which ones are rotated 180
    public static final double bROffset = 226.29;// 227.46;//48.4;
    public static final double DRIVE_TRAIN_WIDTH = 0.57785; // left to right size of drivetrain
    public static final double DRIVE_TRAIN_LENGTH = 0.57785; // front to back size of drivetrain


    public static final double L1_RATIO = 8.14;
    public static final double L2_RATIO = 6.75;
    public static final double L3_RATIO = 6.12;
    public static final double STEER_RATIO = 150.0/7.0;
     public static final double ACTUAL_CIRCUMFERENCE_M = 0.1004; //THIS IS IN METERS 
    public static final double WHEEL_CIRCUMFERENCE = ACTUAL_CIRCUMFERENCE_M * Math.PI; 
   

   
    public static final double ELEVATOR_MIN = 0.005;
    public static final double ELEVATOR_MAX = .20;
    public static final double EXTENSION_MIN = -0.5;
    public static final double EXTENSION_MAX = -95;

    public static final double kMaxVelocity = 6380 * WHEEL_CIRCUMFERENCE / (60.0 * L2_RATIO);
    public static final double kMaxAccel = kMaxVelocity;
    public static final double kMaxAngularVelocity = kMaxVelocity / Math.hypot(DRIVE_TRAIN_LENGTH / 2.0, DRIVE_TRAIN_WIDTH / 2.0);
    public static final double kMaxAngularAccel = kMaxAngularVelocity;
    public static final TrapezoidProfile.Constraints kTurnControlConstraints = new TrapezoidProfile.Constraints(kMaxAngularVelocity, kMaxAngularAccel);
    
    //PID taken from Sonic Squirrels
    //Re-test with full bot
    public static final double kP = 2.2941;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kA = 0.435;
    public static final double kV = 2.344;
    public static final double kS = 0.628;

    public static final double kPThetaController = 2.0; //1.5;
    public static final double kIThetaController = 0.0;
    public static final double kDThetaController = 0.0;
}




