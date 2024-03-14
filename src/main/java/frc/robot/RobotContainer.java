// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeSlightRaise;
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
  public Climber climb = new Climber();
  public SlewRateLimiter slew = new SlewRateLimiter(0.9);
  private final SendableChooser<Command> autoChooser;
  public IntakeSlightRaise slightRaise = new IntakeSlightRaise(this.arm);
  public intakeShoot intakeShoot = new intakeShoot();
  public setSame shoot = new setSame();

  double MAX_RATE = 5.5; // m/s
  double R = Math.sqrt(.5);

  
  public double getDriveJoy(int axis){
    double raw = driveJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.2 ? 0.0 : raw;
  }

  public double getOpJoy(int axis){
    double raw = opJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.2 ? 0.0 : raw;
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


    NamedCommands.registerCommand("SlightRaise", slightRaise);
    NamedCommands.registerCommand("Intake", intakeShoot);
    NamedCommands.registerCommand("Shoot", shoot);

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    leftLaunch.setIdleMode(IdleMode.kBrake);
    rightLaunch.setIdleMode(IdleMode.kBrake);

    intakeSpark.setSmartCurrentLimit(60); 
    leftLaunch.setSmartCurrentLimit(25);
    rightLaunch.setSmartCurrentLimit(25);

    swerveDrive.brake();
    swerveDrive.currentLimit();
    flightSensor.setRangeOfInterest(8, 8, 12, 12);




    //Arm Code 
  
    // joyStick.opButton(3)
    // .onTrue(new InstantCommand(()->arm.setGoal(0)));

    
      
    



    //Launcher Buttons ----------------------------------------------------------------------------------------

    joyStick.opButton(5).whileTrue(new InstantCommand(()->intake()));
    joyStick.opButton(5).whileFalse(new InstantCommand(()->intakeoff()));

    joyStick.opButton(6).onTrue(new ParallelRaceGroup(new setSame(), new WaitCommand(0.4).
    andThen(new ParallelRaceGroup(new intakeShoot(), new WaitCommand(0.2).
    andThen(new ParallelRaceGroup(new off(), new WaitCommand(0.3)))))));

    joyStick.opButton(3).whileTrue(new InstantCommand(()->shootOn()));
    joyStick.opButton(3).whileFalse(new InstantCommand(()->shootOff()));

    joyStick.opButton(7).whileTrue(new InstantCommand(()->climbOut()));
    joyStick.opButton(7).whileFalse(new InstantCommand(()->climbOff()));

    joyStick.opButton(8).whileTrue(new InstantCommand(()->climbIn()));
    joyStick.opButton(8).whileFalse(new InstantCommand(()->climbOff()));

    joyStick.opButton(1).onTrue(new InstantCommand(()->{
      arm.enable();
      arm.setGoal(3.5);
    }));

    joyStick.opButton(4).onTrue(new InstantCommand(()->arm.setGoal(71)));
    
    joyStick.opButton(2).onTrue(new InstantCommand(()->arm.setGoal(90)));
  

 
   //Drive Buttons --------------------------------------------------------------------

  
    joyStick.driveButton(2).onTrue(new InstantCommand(()->pigeon.zeroYaw()));
  

   // SmartDashboard.putData("Straight Auto", new PathPlannerAuto("Straight Path"));
  }

  
  

  


public void teleOperatedInit(){

}





public void intake(){
  intakeSpark.set(-1);
}

public void intakeoff(){
  intakeSpark.set(0);
}

public void shootOff(){
  rightLaunch.set(0);
  leftLaunch.set(0);
}

public void shootOn(){
  rightLaunch.set(0.5);
  leftLaunch.set(-0.5);
  //intakeSpark.set(-1);
}

public void climbOut(){
  climb.climber1.set(0.5);
}

public void climbIn(){
  climb.climber1.set(-0.5);
}

public void climbOff(){
  climb.climber1.set(0);
}




public void teleopPeriodic(){
  double speedRate = SmartDashboard.getNumber("SpeedRate", 1)* MAX_RATE;
  double turnRate = SmartDashboard.getNumber("TurnRate", 0.75)* MAX_RATE/R;

  double xval = getDriveJoy(0)*speedRate; // TODO: CHECK AXIS
  double yval = -getDriveJoy(1)*speedRate;
  double spinval = getDriveJoy(4) * turnRate;

    SmartDashboard.putNumber("TOF", flightSensor.getRange());


    flightSensor.setRangingMode(RangingMode.Short,24);
  
    flightSensor.getRange();

    intakeSpark.setIdleMode(IdleMode.kBrake);
    rightLaunch.setIdleMode(IdleMode.kCoast);
    leftLaunch.setIdleMode(IdleMode.kCoast);

    if (flightSensor.getRange()<=300){
    intakeSpark.set(0);
    leftLaunch.set(-1);
    rightLaunch.set(1);
    opJoy.setRumble(RumbleType.kBothRumble, 0.2);
    }else{
      opJoy.setRumble(RumbleType.kBothRumble, 0);
    }

   swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xval, yval, spinval, pigeon.getAngleDeg()));

  }

  public void roboInit(){
    arm.enable();
    arm.setGoal(60);
    
  }
  public void disableArm(){
    arm.disable();
  }

   private static double convertThrottleInput(double input) {
    double output = ((Constants.THROTTLE_MAX - Constants.THROTTLE_MIN) / 2) * (-input + 1)
                    + Constants.THROTTLE_MIN; // input value is negative because the throttle input is reversed by
    // default;
    return output;
   }

 
  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
    //return new PathPlannerAuto("New Auto");
  }

  }



