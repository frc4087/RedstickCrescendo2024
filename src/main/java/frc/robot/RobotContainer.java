// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final XboxController driveJoy = new XboxController(0);
  public final XboxController opJoy = new XboxController(1);
  public JoystickContainer joyStick = new JoystickContainer(driveJoy,opJoy);
  public ArmSubsystem arm = new ArmSubsystem();
  // public Pigeon2Handler pigeon = new Pigeon2Handler();
  // public SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(pigeon);
  public static double slowmult = 1;

  public double getDriveJoy(int axis){
    double raw = driveJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.1 ? 0.0 : raw;
  }

  public double getOpJoy(int axis){
    double raw = opJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.1 ? 0.0 : raw;
  }

  public double getDriveJoyXR(){
    double raw = getDriveJoy(4);
    return raw; 
  }


  public double getDriveJoyXL(){
    double raw = getDriveJoy(0); //Verify axis
    return raw; 
  }

  public double getDriveJoyYL(){
    double raw = getDriveJoy(1);
    return raw; 
  }

  public double getDriveJoyYR(){
    double raw = getDriveJoy(5);
    return raw;
  }


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

   
    
    // arm code go here
     joyStick.opButton(1)
    .onTrue(new InstantCommand(()->{
      arm.enable();
      arm.setGoal(0);
      }));

     joyStick.opButton(2)
    .onTrue(new InstantCommand(()->arm.setGoal(-60)));

    joyStick.opButton(3)
    .onTrue(new InstantCommand(()->arm.setGoal(-80)));

    joyStick.opButton(4)
    .onTrue(new InstantCommand(()->arm.setGoal(-90)));


  }
  public void init(){
  //arm.enable(); //scary
  arm.setGoal(0);
  }

  
double MAX_RATE = 5.5; // m/s
double R = Math.sqrt(.5);

public void teleOperatedInit(){

}

  public void teleopPeriodic(){
    double intake = -4.5;
    double subwoofer = 0;
    double amp = 99;

    double speedRate = SmartDashboard.getNumber("SpeedRate", 0.3)* MAX_RATE;
    double turnRate = SmartDashboard.getNumber("TurnRate", 1)* MAX_RATE/R;
    SmartDashboard.putNumber("Arm Abs Enc", arm.getArmEncoderPos());

    //SmartDashboard.putNumber("driveJoyXR", getDriveJoyXR());
    SmartDashboard.putNumber("drivejoyYL", getDriveJoyYL());

    double xval = getDriveJoyXR()*speedRate; // TODO: CHECK AXIS
    double yval = -getDriveJoyYR()*speedRate;
    double spinval = getDriveJoyXL() * turnRate;

   

     //swerveDrive.drive(new ChassisSpeeds(xval, yval, spinval));
    // swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xval, yval, spinval, pigeon.getAngleDeg()));

    // AButton.onTrue(new InstantCommand(()->pigeon.zeroYaw()));


    // if(opJoy.getBButton()){
    //   arm.setGoal(intake);
    //   arm.enable();
    // }else if(opJoy.getYButton()){
    //   arm.setGoal(amp);
    //   arm.enable();
    // }else if(opJoy.getXButton()){
    //   arm.setGoal(subwoofer);
    //   arm.enable();
    // }
    
  }

  public void disableArm(){
    arm.disable();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   private static double convertThrottleInput(double input) {
    double output = ((Constants.THROTTLE_MAX - Constants.THROTTLE_MIN) / 2) * (-input + 1)
                    + Constants.THROTTLE_MIN; // input value is negative because the throttle input is reversed by
    // default;
    return output;
   }
}
