// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WinchSubsystem;

public class WinchRetractCmd extends Command {
  WinchSubsystem winchSubsystem;
  /** Creates a new WinchRetractCmd. */
  public WinchRetractCmd(WinchSubsystem winchSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.winchSubsystem = winchSubsystem;
    addRequirements(winchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    winchSubsystem.retractLift();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    winchSubsystem.liftStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
