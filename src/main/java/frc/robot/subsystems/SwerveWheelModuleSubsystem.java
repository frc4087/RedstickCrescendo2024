// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.util.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveWheelModuleSubsystem extends SubsystemBase {
  double P = 0.00675;//0.000072;//0.000081;
  double I = 0.00001;//0.000007;
  double D = 0;//0.0000065;
  double GEAR_RATIO = 0.148; //6.75 to 1 whatever that means
  //hard code in the actual values once we find them off of smartdashboard

  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private PIDController pidController;
  //private CANCoder angleEncoder;
  private CANcoder angleEncoder;
  //private boolean calibrateMode;
  private double encoderOffset;
  private String motorName;
  private CANcoderConfiguration angleEncoderConfig;
  private int counter = 0;
  //private SimpleWidget speedLim;

  public SwerveWheelModuleSubsystem(int angleMotorChannel, int speedMotorChannel, int angleEncoderChannel,
          String motorName, double offset) {
      // We're using TalonFX motors on CAN.
      this.angleMotor = new CANSparkMax(angleMotorChannel,  MotorType.kBrushless);
      this.speedMotor = new CANSparkMax(speedMotorChannel,  MotorType.kBrushless);
      this.angleEncoder = new CANcoder(angleEncoderChannel); // CANCoder Encoder //TODO: CHECK THIS RANGE IS 0-360
      this.speedMotor.setIdleMode(IdleMode.kCoast);
      this.motorName = motorName;
      this.pidController = new PIDController(P, I, D); // This is the PID constant,
      // we're not using any
      // Integral/Derivative control but increasing the P value will make
      // the motors more aggressive to changing to angles.
      
      
      //angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

      // pidController.setTolerance(20); //sets tolerance, shouldn't be needed.

      pidController.enableContinuousInput(0, 360);// This makes the PID controller
      // understand the fact that for
      // our setup, 360 degrees is the same as 0 since the wheel loops.
        
      angleEncoderConfig = new CANcoderConfiguration();
      angleEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
      angleEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      angleEncoder.getConfigurator().apply(angleEncoderConfig);
      //CANcoders may report 

      SendableRegistry.addChild(this, angleMotor);
      SendableRegistry.addChild(this, speedMotor);
      SendableRegistry.addChild(this, angleEncoder);
      SendableRegistry.addLW(this, "Swerve Wheel Module");
      //create smartDashboard widget for encoder offset

      encoderOffset = offset;
      //UNCOMMENT TO DO AUTO CALIBRATION
      // double startAngle = angleEncoder.getAbsolutePosition();
      // encoderOffset = MathUtil.mod(startAngle - offset, 360);
      // SmartDashboard.putNumber("Encoder Offset " + motorName, encoderOffset);

      //speedLim = Shuffleboard.getTab("Preferences").addPersistent("Speed Lim", 0.5)
      //.withWidget(BuiltInWidgets.kNumberSlider);;
      
      // resetSensor();


  }
  public void setAngle(double angle, double currentEncoderValue)
  {
      SmartDashboard.putNumber("Difference " + motorName, MathUtil.getCyclicalDistance(currentEncoderValue, angle, 360));
      angle = MathUtil.mod(angle, 360);

      
      
      double pidOut = -pidController.calculate(currentEncoderValue, angle);
    
      
      angleMotor.set(pidOut);
      SmartDashboard.putNumber("PID Out " + motorName, pidOut);

      
  }

  public void setSpeed(double speed)
  {
      speedMotor.set(speed/5.5); //why 5.5? 
  }

  
  public void drive(double speed, double angle) {
      double currentEncoderValue = getPosition();
      
      // SmartDashboard.putNumber("Distance " + motorName, getDistance());
      SmartDashboard.putNumber("Rotation " + motorName, getPosition());
      SmartDashboard.putNumber("angle reading", angle);
      SmartDashboard.putNumber("Cyclical Distance", MathUtil.getCyclicalDistance(currentEncoderValue, angle, 360));
   

      if(MathUtil.getCyclicalDistance(currentEncoderValue, angle, 360) > 90){
        setSpeed(-speed);
        setAngle(angle+180, currentEncoderValue);
      }else{
        setSpeed(speed);
        setAngle(angle, currentEncoderValue);
      }
    
  }




 
  // this method outputs position of the encoder to the smartDashBoard, useful for
  // calibrating the encoder offsets

   


    public double getPosition(){
       return MathUtil.mod(angleEncoder.getAbsolutePosition().refresh().getValue()*360 - encoderOffset, 360);
    }

  // public double getPositionRad() {
  //     return MathUtil.mod(angleEncoder.getAbsolutePosition().refresh().getValue()*2*Math.PI - encoderOffset, 360) * Math.PI / 180;
  // }
  
  public double getPositionRad(){
    return getPosition()*Constants.degToRad;
  }

  public double getDistance() {
      if (motorName.equals("BR") || motorName.equals("FR")) {
          return -(speedMotor.getEncoder().getPosition()* Constants.WHEEL_CIRCUMFERENCE)/(2048 * Constants.L2_RATIO); 
      }
      return (speedMotor.getEncoder().getPosition() * Constants.WHEEL_CIRCUMFERENCE)/(2048 * Constants.L2_RATIO);
      //2048 value is from falcon encoder distance per pulse
    }

  public void stop() {
      speedMotor.set(0);
      angleMotor.set(0);
  }

  @Override
  public void periodic() {
      //calibrateMode = calibrateState.getEntry().getBoolean(false);
      SmartDashboard.putNumber("speed Encoder " + motorName, speedMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("angle Encoder " + motorName, angleMotor.getEncoder().getPosition());
      counter++;
      SmartDashboard.putNumber("counter", counter);
      

      P = SmartDashboard.getNumber("P", P);
      I = SmartDashboard.getNumber("I", I);
      D = SmartDashboard.getNumber("D", D);
      pidController.setPID(P, I, D);
    
      SmartDashboard.putNumber("Encoder (Calculated) " + motorName, getPosition());
      SmartDashboard.putNumber("raw Cancoder "+ motorName, angleEncoder.getAbsolutePosition().refresh().getValueAsDouble());

  }

  public void coast(){
      speedMotor.setIdleMode(IdleMode.kCoast);
  }

  public void brake(){
      speedMotor.setIdleMode(IdleMode.kBrake);
  }

  public void swerveCurrentLimit(){
    speedMotor.setSmartCurrentLimit(40);
    angleMotor.setSmartCurrentLimit(40);
  }

  private double getSpeedMotorSpeed(){
    return this.speedMotor.getEncoder().getVelocity()*GEAR_RATIO; 
    //TODO: need to convert RPM of motor, to Translational speed of wheel (meters per second)
  }

  public double getSpeedMotorPosition(){
    return this.speedMotor.getEncoder().getPosition()*GEAR_RATIO*Constants.WHEEL_CIRCUMFERENCE; 
    //this didn't work 
  }


  public SwerveModuleState getSwerveModuleState(){
    double speed = getSpeedMotorSpeed();
    Rotation2d angle = new Rotation2d(this.getPosition());
    return new SwerveModuleState(speed,angle);
  }

  public SwerveModulePosition getSwerveModulePosition(){
    double position = getSpeedMotorPosition();
    Rotation2d angle =  new Rotation2d(this.getPosition());
    return new SwerveModulePosition(position,angle);
    
  }
//   public void resetSensor()
//   {
//       speedMotor.getSelectedSensorPosition(0);
//   }

  

}
