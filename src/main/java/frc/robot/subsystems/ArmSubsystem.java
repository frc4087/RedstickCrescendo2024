// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  public final CANSparkMax m_spark = new CANSparkMax(11,MotorType.kBrushless);
  public final CANSparkMax m_spark2 = new CANSparkMax(12, MotorType.kBrushless);
  //public final RelativeEncoder encoder_arm = m_spark.getAlternateEncoder();
  //public final CANSparkMax hand_spark = new CANSparkMax(21,MotorType.kBrushless);
  public final SparkAbsoluteEncoder alternateEncoder;
  public final ArmFeedforward ff = new ArmFeedforward(0, 0.03, 0, 0);
  //public final DutyCycleEncoder dutyCycleEncoder;
  //private final double ENCODER_OFFSET = -1.416992;
  //encoder_arm.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);

  
  /** Creates a new Arm. */

  //public final CANSparkMax spark = new CANSparkMax (14, MotorType.kBrushless);
 // public final RelativeEncoder encoderS = spark.getEncoder();
    
  
  public ArmSubsystem() {


    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0.03,
            0.0007,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(200, 200)));
    this.alternateEncoder = m_spark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  
    m_spark.setIdleMode(IdleMode.kBrake);
    m_spark2.setIdleMode(IdleMode.kBrake);
    m_spark.setSmartCurrentLimit(40);
    m_spark2.setSmartCurrentLimit(40);
    
    m_spark2.follow(m_spark,true);


  } // what is Kdt?

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
      // double invertedOutput = output * -1;
    double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
  SmartDashboard.putNumber("Arm Output", output);
  // SmartDashboard.putNumber("Inverted Arm Output", invertedOutput);
   m_spark.set(output + feedforward);
   //m_spark2.set(output);
 
  // hand_spark.set(output);
  }


  // double kgOutput = Math.cos(setpoint.position);
  // m_spark.set(kgOutput + output);

   public double getArmEncoderPos(){ // arm deg

   return (-alternateEncoder.getPosition()*360); //Put encoder offset back in 

 }



  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getArmEncoderPos();
    
    
  }
}
