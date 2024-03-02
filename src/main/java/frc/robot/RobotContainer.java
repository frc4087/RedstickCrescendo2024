// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveDriveSubsystem;
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
  public JoystickContainer joySticks = new JoystickContainer(driveJoy,opJoy);
  public Pigeon2Handler pigeon = new Pigeon2Handler();
  public SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(pigeon);
  public static double slowmult = 1;
  private final SendableChooser<Command> autoChooser;


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
    return Math.abs(raw) < 0.1 ? 0.0 : raw; 
 
  }


  public double getDriveJoyXL(){
    double raw = getDriveJoy(0); //Verify axis
    return Math.abs(raw) < 0.1 ? 0.0 : raw; 
  }

  public double getDriveJoyYL(){
    double raw = getDriveJoy(1);
    return Math.abs(raw) < 0.1 ? 0.0 : raw; 
 
  }

  public double getDriveJoyYR(){
    double raw = getDriveJoy(5);
    return Math.abs(raw) < 0.1 ? 0.0 : raw; 

  }

  public Trajectory trajectory;
  public Command m_autonomousCommand;




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    joySticks.driveButton(1).onTrue(new InstantCommand(()->pigeon.zeroYaw()));
    autoChooser = AutoBuilder.buildAutoChooser();

  

  }

  
double MAX_RATE = 5.5; // m/s TODO: Drag race the robot
double R = Math.sqrt(.5);
  public void teleopPeriodic(){
    double speedRate = SmartDashboard.getNumber("SpeedRate", 0.3)* MAX_RATE; //push controller to 1 -> Max speed (MAX_RATE).
    double turnRate = SmartDashboard.getNumber("TurnRate", 1)* MAX_RATE/R;
    SmartDashboard.putNumber("Front Right", swerveDrive.frontRight.getPosition());
    SmartDashboard.putNumber("Front Left", swerveDrive.frontLeft.getPosition());
    SmartDashboard.putNumber("Back Right", swerveDrive.backRight.getPosition());
    SmartDashboard.putNumber("Back Left", swerveDrive.backLeft.getPosition());

    //SmartDashboard.putNumber("driveJoyXR", getDriveJoyXR()
    SmartDashboard.putNumber("drivejoyYL", getDriveJoyYL());

    double xval = getDriveJoyXR()*speedRate; // m/s
    double yval = -getDriveJoyYR()*speedRate;// m/s
    double spinval = getDriveJoyXL() * turnRate; // rad/s

    //  swerveDrive.drive(new ChassisSpeeds(xval, yval, spinval));
    swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xval, yval, spinval, pigeon.getAngleDeg()));
    
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

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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

    

    
  //   // Run path following command, then stop at the end.
  //   // Robot.m_robotContainer.m_driveAuto.m_drive.feed();
  //   //m_drivebase.resetOdometry(trajectory.getInitialPose());
    
  //   if (!multiPath){
  //     swerveDrive.resetPose();
  //   } 
  // }



}
