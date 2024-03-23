// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class Constants {
    public static final double THROTTLE_MIN = 0.4;
    public static final double THROTTLE_MAX = 1.0;
    public static final double MAX_ANGULAR_SPEED_RADIANS = Math.PI; //change 
    public static final double MAX_ANGULAR_SPEED_RADIANS_SQ = Math.PI; //change 
    public static final double GEAR_RATIO = 0.148;
    //DriveBase Constants - Unchecked 
    public static final double ksVolts = 0.18531,//0.65634, 
                                kvVoltSecondsPerMeter = 1.0502, //2.6376,  //0.1106, 
                                kaVoltSecondsSquaredPerMeter = 0.13501,//1.15 //0.095387,
                                kTrackwidthMeters = 0.514, //ChargedUp Update
                                kP = 1,//0.17833, 
                                kD = 0.0, 
                                kMaxSpeedMetersPerSecond = 4.6634, //ChargedUp
                                kMaxAccelerationMetersPerSecondSquared = 5,
                                kRamseteB = 2, 
                                kRamseteZeta = 0.7,
                                K_P = 1;//change;

    public static final TrapezoidProfile.Constraints K_THETA =
        new TrapezoidProfile.Constraints(
           MAX_ANGULAR_SPEED_RADIANS, MAX_ANGULAR_SPEED_RADIANS_SQ);
}
