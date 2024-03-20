// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FullShoot extends Command {
  /** Creates a new FullShoot. */
  public FullShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_robotContainer.intakeSpark.set(-1);
    Robot.m_robotContainer.leftLaunch.set(-1);
    Robot.m_robotContainer.rightLaunch.set(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.intakeSpark.set(0);
    Robot.m_robotContainer.leftLaunch.set(0);
    Robot.m_robotContainer.rightLaunch.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
