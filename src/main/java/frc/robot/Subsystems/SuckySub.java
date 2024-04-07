// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import frc.robot.Constants.Ports;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuckySub extends SubsystemBase {

  private CANSparkMax sucky = new CANSparkMax(Ports.sucky, MotorType.kBrushless);
  boolean suckyOn = false;
  public SuckySub() {}

  public void sucky(Boolean LB, Boolean RB) {
 
    if (LB) {
      suckyOn = true;
      sucky.set(-.95);
    } 
    else if (RB) {
      suckyOn = true;
      sucky.set(0.8);
    } 
    else {
      suckyOn = false;
      sucky.set(0);
    }

  }


  public void suckyStop() {
    sucky.set(0);
  }

  public boolean suckyOn(){
    return suckyOn;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
