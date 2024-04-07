// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSub;
import frc.robot.Subsystems.IndexSub;
import frc.robot.Subsystems.ShootaSub;

public class AutoDrive extends Command {
  /** Creates a new AutoDrive. */
  private ShootaSub shootaSub;
  private DriveSub driveSub;
  private IndexSub indexSub;
  boolean finished = false;
  int i=0;
   
  
  public AutoDrive(ShootaSub ss, DriveSub ds, IndexSub is) {
    // Use addRequirements() here to declare subsystem dependencies.
    shootaSub = ss;
    driveSub = ds;
    indexSub = is;
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    finished = false;
    i=0;
    shootaSub.resetEncoder();
    indexSub.resetEncoder();
    driveSub.encoderReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // while(driveSub.onNotTarget()){
    //   driveSub.AptMove(driveSub.AptXDistance());
    //  }
  while(shootaSub.GetEncoder()<100){
       shootaSub.AutoShoot();
  }
   while(indexSub.GE()>-25){
       indexSub.AutoShoot();
   }
      indexSub.AutoStop();
      shootaSub.ShootStop();
  while(i<20000 ){
    i++;
       driveSub.AutoDrive();
  }
       finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
