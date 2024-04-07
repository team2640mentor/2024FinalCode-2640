// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IndexSub;
import frc.robot.Subsystems.SuckySub;

public class SuckyCom extends Command {
  //private IndexSub indexSub = new IndexSub();
  private SuckySub suckySub;
  private BooleanSupplier LB;
  private BooleanSupplier RB;
   
  public SuckyCom(SuckySub sus, BooleanSupplier L, BooleanSupplier R) {
  suckySub=sus;
LB=L;
RB=R;
suckySub=sus;
addRequirements(suckySub); 
//addRequirements(indexSub);
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//     if (LB.getAsBoolean() || RB.getAsBoolean()){
// //indexSub.indexSucky(LB.getAsBoolean(),RB.getAsBoolean());
// suckySub.sucky(LB.getAsBoolean(),RB.getAsBoolean());
// SmartDashboard.putBoolean("lb", LB.getAsBoolean());
// SmartDashboard.putBoolean("rb", RB.getAsBoolean());
    //}

    suckySub.sucky(LB.getAsBoolean(), RB.getAsBoolean());
   
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
