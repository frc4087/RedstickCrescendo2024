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

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveWheelModuleSubsystem extends SubsystemBase {
  double TURN_P = 0.00675;//0.000072;//0.000081;
  double TURN_I = 0.00001;//0.000007;
  double TURN_D = 0;//0.0000065;

  double SPEED_P = 1; //TODO: TUNE Velocity PID Controller     (currentVal, Target value) ex:(0m/s, 5m/s) 
  double SPEED_I = 0;
  double SPEED_D = 0;

  double GEAR_RATIO = 2;
  //hard code in the actual values once we find them off of smartdashboard

  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private PIDController turnPIDController;
  private PIDController speedPIDController;
  //private CANCoder angleEncoder;
  private CANcoder angleEncoder;
  //private boolean calibrateMode;
  private double encoderOffset;
  private String motorName;
  //private SimpleWidget speedLim;

  public SwerveWheelModuleSubsystem(int angleMotorChannel, int speedMotorChannel, int angleEncoderChannel,
          String motorName, double offset) {
      // We're using TalonFX motors on CAN.
      this.angleMotor = new CANSparkMax(angleMotorChannel,  MotorType.kBrushless);
      this.speedMotor = new CANSparkMax(speedMotorChannel,  MotorType.kBrushless);
      this.angleEncoder = new CANcoder(angleEncoderChannel); // CANCoder Encoder //TODO: CHECK THIS RANGE IS 0-360
      this.speedMotor.setIdleMode(IdleMode.kCoast);
      this.motorName = motorName;
      this.turnPIDController = new PIDController(TURN_P, TURN_I, TURN_D); // This is the PID constant,
      this.speedPIDController = new PIDController(SPEED_P,SPEED_I,SPEED_D);
      // we're not using any
      // Integral/Derivative control but increasing the P value will make
      // the motors more aggressive to changing to angles.
      
      

      //angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

      // pidController.setTolerance(20); //sets tolerance, shouldn't be needed.

      turnPIDController.enableContinuousInput(0, 360); // This makes the PID controller
      // understand the fact that for
      // our setup, 360 degrees is the same as 0 since the wheel loops.

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

      
      
      double pidOut = -turnPIDController.calculate(currentEncoderValue, angle);
    
      
      angleMotor.set(pidOut);
      SmartDashboard.putNumber("PID Out " + motorName, pidOut);

      
  }

  public void setSpeed(double speedMetersPerSecond)
  {
      speedPIDController.setSetpoint(speedMetersPerSecond);
  }

  
  public void drive(double speed, double angle) {
      double currentEncoderValue = getPosition();
      
      // SmartDashboard.putNumber("Distance " + motorName, getDistance());
      SmartDashboard.putNumber("Rotation " + motorName, getPositionRad());
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

  public double getPositionRad() {
      return MathUtil.mod(angleEncoder.getAbsolutePosition().refresh().getValue()*2*Math.PI - encoderOffset, 360) * Math.PI / 180;
  }

  public double getDistance() {
      if (motorName.equals("BR") || motorName.equals("FR")) {
          return -(speedMotor.getEncoder().getPosition()* Constants.WHEEL_CIRCUMFERENCE)/(2048 * Constants.L2_RATIO);
      }
      return (speedMotor.getEncoder().getPosition() * Constants.WHEEL_CIRCUMFERENCE)/(2048 * Constants.L2_RATIO);
  }

  public void stop() {
      speedMotor.set(0);
      angleMotor.set(0);
  }

  @Override
  public void periodic() {
      //calibrateMode = calibrateState.getEntry().getBoolean(false);

      // update values on the fly, uncomment when tuning
      // TURN_P = SmartDashboard.getNumber("P", TURN_P);
      // TURN_I = SmartDashboard.getNumber("I", TURN_I);
      // TURN_D = SmartDashboard.getNumber("D", TURN_D);
      // turnPIDController.setPID(TURN_P, TURN_I, TURN_D);
      // SmartDashboard.putNumber("Encoder " + motorName, getPosition());

    double speedThrottleValue = speedPIDController.calculate(getSpeedMotorSpeed());
    speedMotor.set(speedThrottleValue);


  }

  public void coast(){
      speedMotor.setIdleMode(IdleMode.kCoast);
  }

  public void brake(){
      speedMotor.setIdleMode(IdleMode.kBrake);
  }

  private double getSpeedMotorSpeed(){
    return this.speedMotor.getEncoder().getVelocity()*GEAR_RATIO; 
    //TODO: need to convert RPM of motor, to Translational speed of wheel (meters per second)
  }

  private double getSpeedMotorPosition(){
    return this.speedMotor.getEncoder().getPosition()*GEAR_RATIO;
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
