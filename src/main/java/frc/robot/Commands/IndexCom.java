// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IndexSub;

public class IndexCom extends Command {
  private IndexSub indexSub;
  private BooleanSupplier LB;
  private BooleanSupplier RB;
  private BooleanSupplier Sensor;
  private BooleanSupplier yButton;
  private BooleanSupplier trig;
  private BooleanSupplier eight;
  private BooleanSupplier svn;
  private BooleanSupplier two;


  public IndexCom(IndexSub IS, BooleanSupplier t, BooleanSupplier L, BooleanSupplier R, BooleanSupplier Y,
   BooleanSupplier Sens, BooleanSupplier et , BooleanSupplier sv, BooleanSupplier tw) {
    indexSub = IS;
    trig = t;
    LB = L;
    RB = R;
    yButton= Y;
    Sensor = Sens;
    eight = et;
    svn = sv;
    two = tw;
    addRequirements(indexSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
SmartDashboard.putBoolean("NOTE IN", indexSub.HasTripped() );
   if(eight.getAsBoolean()){
    indexSub.trapHo();
   } else
    indexSub.indexSucky(LB.getAsBoolean(), RB.getAsBoolean(), Sensor.getAsBoolean(),trig.getAsBoolean(), svn.getAsBoolean(), two.getAsBoolean());

  if(yButton.getAsBoolean()){
      indexSub.resetRB();
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
