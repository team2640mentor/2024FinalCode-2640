// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IndexSub;

public class AutoIndex extends Command {
  private IndexSub indexSub;
  /** Creates a new AutoIndex. */
  public AutoIndex(IndexSub is) {
    indexSub = is;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexSub.AutoShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexSub.AutoStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexSub.GE() > -25;
  }
}