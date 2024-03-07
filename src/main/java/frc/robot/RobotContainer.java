// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Consumer;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm2;
//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.intakeShoot;
import frc.robot.subsystems.off;
import frc.robot.subsystems.setSame;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

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
  //public ArmSubsystem arm = new ArmSubsystem();
  public Arm2 arm = new Arm2();
  public Pigeon2Handler pigeon = new Pigeon2Handler();
  public SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(pigeon);
  public static double slowmult = 1;
  public TimeOfFlight flightSensor = new TimeOfFlight(40);
  public CANSparkMax rightLaunch = new CANSparkMax(30,MotorType.kBrushless);
  public CANSparkMax leftLaunch = new CANSparkMax(31,MotorType.kBrushless);
  public CANSparkMax intakeSpark = new CANSparkMax(32,MotorType.kBrushless);

  
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

public Trajectory trajectory;
  public Command m_autonomousCommand;
  public SendableChooser<String> autoChooser = new SendableChooser<String>();
  public Command Shoot;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

   // SmartDashboard.putNumber("voltage", arm.getVoltage());

    joyStick.opButton(1)
    .onTrue(new InstantCommand(()->{
      arm.enable();
      arm.setGoal(5);
    }));
    
    //arm code go here
     joyStick.opButton(2)
    .onTrue(new InstantCommand(()->arm.setGoal(60)));
  
    joyStick.opButton(3)
    .onTrue(new InstantCommand(()->arm.setGoal(30)));

    flightSensor.setRangeOfInterest(8, 8, 12, 12);

    joyStick.opButton(5).onTrue(new InstantCommand(()->intake()));

    joyStick.opButton(6).onTrue(new ParallelRaceGroup(new setSame(), new WaitCommand(0.8).
    andThen(new ParallelRaceGroup(new intakeShoot(), new WaitCommand(0.5).
    andThen(new ParallelRaceGroup(new off(), new WaitCommand(0.3)))))));

    // joyStick.opButton(4)
    // .onTrue(new InstantCommand(()->arm.setGoal(90)));

    joyStick.opButton(5).onFalse(new InstantCommand(()->intakeoff()));
   // joySticks.opButton(1).onFalse(new InstantCommand(()->shootOff()));
  

 

  
   // joySticks.driveButton(1).onTrue(new InstantCommand(()->pigeon.zeroYaw()));
  }

  
  

  
double MAX_RATE = 5.5; // m/s
double R = Math.sqrt(.5);

public void teleOperatedInit(){

}





public void intake(){
  intakeSpark.set(-0.8);
  leftLaunch.set(-0.5);
  rightLaunch.set(0.5);
}

public void intakeoff(){
  intakeSpark.set(0);
}

public void shootOff(){
  rightLaunch.set(0);
  leftLaunch.set(0);
}

public void teleopPeriodic(){
  double speedRate = SmartDashboard.getNumber("SpeedRate", 0.1)* MAX_RATE;
  double turnRate = SmartDashboard.getNumber("TurnRate", .2)* MAX_RATE/R;
   double intake = -4.5;
    double subwoofer = 0;
    double amp = 99;
  // SmartDashboard.putNumber("Front Right", swerveDrive.frontRight.getPosition());
  //   SmartDashboard.putNumber("Front Left", swerveDrive.frontLeft.getPosition());
  //   SmartDashboard.putNumber("Back Right", swerveDrive.backRight.getPosition());
  //   SmartDashboard.putNumber("Back Left", swerveDrive.backLeft.getPosition());
  SmartDashboard.putNumber("TOF", flightSensor.getRange());


   flightSensor.setRangingMode(RangingMode.Short,24);
  
    flightSensor.getRange();

    intakeSpark.setIdleMode(IdleMode.kBrake);
    rightLaunch.setIdleMode(IdleMode.kCoast);
    leftLaunch.setIdleMode(IdleMode.kCoast);

    if (flightSensor.getRange()<=150){
    intakeSpark.set(0);
    }


 


  //joySticks.opButton(1).whileFalse(new InstantCommand(()->shootOff()));
 // joySticks.opButton(2).onFalse(new InstantCommand(()->intakeoff()));


  SmartDashboard.putNumber("drivejoyYL", getDriveJoyYL());

  double xval = getDriveJoyXR()*speedRate; // TODO: CHECK AXIS
  double yval = -getDriveJoyYR()*speedRate;
  double spinval = getDriveJoyXL() * turnRate;


    //  swerveDrive.drive(new ChassisSpeeds(xval, yval, spinval));
  swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xval, yval, spinval, pigeon.getAngleDeg()));

  

    
}
  public void roboInit(){
    
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

 
  
  //  public Command pathFollow(String trajectoryJSON, boolean multiPath){

  //    double kPXControl = 1; //change
  //   double kPYControl = 1; //change
  //   ProfiledPIDController thetaController = new ProfiledPIDController(Constants.K_P, 0, 0, Constants.K_THETA);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);


  //   try {
  //     Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  //     trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
  //   } catch (final IOException ex) {
  
  
  //     DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  //   }
  //   //m_drivebase.m_gyro.reset();

  //   Consumer<SwerveModuleState[]> moduleStateConsumer = (states) -> swerveDrive.setSwerveModuleStates(states);

  //   SwerveControllerCommand swerveController = new SwerveControllerCommand(
  //   trajectory,
  //   swerveDrive.getRobotPoseSupplier(), 
  //   swerveDrive.getKinematics(),
  //   new PIDController(kPXControl, 0, 0),
  //   new PIDController(kPYControl, 0, 0),
  //   thetaController,
  //   moduleStateConsumer,
  //   swerveDrive);

    

    
  //   // Run path following command, then stop at the end.
  //   // Robot.m_robotContainer.m_driveAuto.m_drive.feed();
  //   //m_drivebase.resetOdometry(trajectory.getInitialPose());
    
  //   if (!multiPath){
  //     swerveDrive.resetPose();
  //   } 
  //   return swerveController;
  // }



}
