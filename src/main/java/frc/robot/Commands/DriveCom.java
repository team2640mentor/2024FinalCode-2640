// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.DriveSub;

public class DriveCom extends Command {
  /** Creates a new DriveCom. */
  DriveSub driveSub;
  DoubleSupplier translateSupplier;
  DoubleSupplier rotateSupplier;
  BooleanSupplier bButton;
  double i = 0;
  /* AptTest */Double xDis;
  double ty;
  double tr;
  double rr;
  double FT = LimelightHelpers.getFiducialID("limelight-driver");
  boolean tv = LimelightHelpers.getTV("limelight-driver");
  BooleanSupplier shootaOn;
  BooleanSupplier indexOn;
  BooleanSupplier armOn;
  BooleanSupplier abutt;
  // boolean b = true;

  public DriveCom(DriveSub d, DoubleSupplier t, DoubleSupplier r, BooleanSupplier b,
  BooleanSupplier SO, BooleanSupplier IO, BooleanSupplier AO, BooleanSupplier aB) {
    driveSub = d;
    translateSupplier = t;
    rotateSupplier = r;
    bButton = b;
    shootaOn = SO;
    indexOn =IO;
    armOn = AO;
    abutt = aB;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSub.encoderReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //driveSub.AutoDrive();
     SmartDashboard.putNumber("FT", FT);
   
SmartDashboard.putBoolean("OT", driveSub.onNotTarget());
    xDis = driveSub.AptXDistance();
     SmartDashboard.putNumber("TX2", xDis);

  if (bButton.getAsBoolean() && xDis!=0) {
      while (driveSub.onNotTarget()) {
        SmartDashboard.putBoolean("NOT", driveSub.onNotTarget());

        driveSub.AptMove(driveSub.AptXDistance());

 SmartDashboard.putNumber("TXX", xDis);
        //SmartDashboard.putString("I ran through (ur mom edition)", "Si caso");

      }
    }
    // break or smthn
    SmartDashboard.putNumber("RealV", driveSub.getRealSpeeds());
    SmartDashboard.putNumber("LFEP", driveSub.getLeftEncoderPos());
    SmartDashboard.putNumber("RFEP", driveSub.getRightEncoderPos());
    SmartDashboard.putNumber("Velocity", driveSub.getSpeeds());
    i = i + .001;
    if(abutt.getAsBoolean()){
     tr = .75*(translateSupplier.getAsDouble());
     rr  = .75*(rotateSupplier.getAsDouble());
  }else
     tr = 1*(translateSupplier.getAsDouble());
     rr  = 1*(rotateSupplier.getAsDouble());
     
    driveSub.dDrive.arcadeDrive((tr), (rr));

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
