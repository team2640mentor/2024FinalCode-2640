// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSub;

public class ClimberCom extends Command {
ClimberSub climberSub;
BooleanSupplier climbSet;
BooleanSupplier climb;
  public ClimberCom(ClimberSub cSub, BooleanSupplier cSet, BooleanSupplier c) {
    climberSub = cSub;
    climbSet = cSet;
    climb = c;
addRequirements(cSub); 
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   climberSub.climber(climb.getAsBoolean(), climbSet.getAsBoolean());
   SmartDashboard.putNumber("CLIMBER",climberSub.encoderPos());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
