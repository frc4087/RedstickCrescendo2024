// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeSlightRaise extends Command {
  private Arm2 arm;
  
  /** Creates a new IntakeSlightRaise. */
  public IntakeSlightRaise(Arm2 arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.arm.enable();
    this.arm.setGoal(5);

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.arm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
