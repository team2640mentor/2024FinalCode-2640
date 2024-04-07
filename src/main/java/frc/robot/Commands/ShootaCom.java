// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShootaSub;

public class ShootaCom extends Command {
  public ShootaSub shootaSub;
  private BooleanSupplier trig;

  double i = 0;
  DigitalOutput led = new DigitalOutput(0);

  public ShootaCom(ShootaSub ss, BooleanSupplier t) {
    // fourteen large men grasping upon my large big toe
    shootaSub = ss;

    trig = t;

    led.set(!led.get());
    //addRequirements(Ports.indexSub);
    addRequirements(shootaSub);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootaSub.shoota(trig.getAsBoolean());

   
    i = i+1;
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
