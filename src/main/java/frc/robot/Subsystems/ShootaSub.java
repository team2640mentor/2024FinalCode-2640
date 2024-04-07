// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//import frc.robot.Constants.EncoderConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.LimelightHelpers;
import frc.robot.Constants.Ports;

public class ShootaSub extends SubsystemBase {
  /** Creates a new Shoota. */
  private CANSparkMax topShoota = new CANSparkMax(Ports.topShoot, MotorType.kBrushless);
  private CANSparkMax bottomShoota = new CANSparkMax(Ports.bottomShoot, MotorType.kBrushless);
  private RelativeEncoder SE = topShoota.getEncoder();
  boolean shootaOn = false;
public ShootaSub() {
    bottomShoota.follow(topShoota);
    SE.setPosition(0);
}

public void shoota(Boolean trig) {
    if (trig) {
      shootaOn = true;
     
      topShoota.set(.99);
    } else {
      shootaOn = false;
      topShoota.set(0);
    }
}


public void AutoShoot() {
    //  while (SE.getPosition() < 75) {//k
      topShoota.set(.8);
      
    //  }//k

}


public void resetEncoder(){
      SE.setPosition(0);
}

public void ShootStop() {
    topShoota.set(0);
}

public double GetEncoder(){
  return SE.getPosition();
}

public boolean shootaOn(){
  return shootaOn;
}

  @Override
public void periodic() {
    // This method will be called once per scheduler run
}
}
