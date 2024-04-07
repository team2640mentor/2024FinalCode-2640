// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShootaSub;

public class AutoShoot extends Command {
  private ShootaSub shootaSub;
  boolean ppp; 
  /** Creates a new AutoShoot. */
  public AutoShoot(ShootaSub ss) {
    shootaSub = ss;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      ppp= false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ppp=true;
    SmartDashboard.putBoolean("ppoooo", ppp);
    shootaSub.AutoShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootaSub.ShootStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shootaSub.GetEncoder() < 75;
  }
}
