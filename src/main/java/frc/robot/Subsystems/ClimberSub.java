// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class ClimberSub extends SubsystemBase {
  private CANSparkMax climber = new CANSparkMax(Ports.climber, MotorType.kBrushless);
  private RelativeEncoder CE = climber.getEncoder();
  boolean climberOn  = false;
  /** Creates a new Climber. */
  public ClimberSub() {
    CE.setPosition(0);
  }

  public void climber(Boolean climb, Boolean climbSet) {
    if (climbSet) {
      climberOn = true;
      climber.set(.25);

    } else if (climb) {
      climberOn = true;
      climber.set(-.5);
    } else {
      climberOn = false;
      climber.set(0);
    }
  }
  public double encoderPos(){
    return CE.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
