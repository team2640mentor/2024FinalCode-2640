// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSub;

public class ArmCom extends Command {
  /** Creates a new NoteMoveCom. */
  private ArmSub armSub;
  private BooleanSupplier two;
  private BooleanSupplier three;
  private BooleanSupplier six;
  private BooleanSupplier svn;
  private BooleanSupplier eit;
  private BooleanSupplier bButton;
  private BooleanSupplier four;
  private BooleanSupplier five;


  public ArmCom(ArmSub a, BooleanSupplier b, BooleanSupplier sx, BooleanSupplier sv, BooleanSupplier t,
      BooleanSupplier tre, BooleanSupplier e, BooleanSupplier f, BooleanSupplier fi) {
    armSub = a;
    bButton = b;
    two = t;
    three = tre;
    six = sx;
    svn = sv;
    eit = e;
    four = f;
    five = fi;

    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSub.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (six.getAsBoolean()) {
      armSub.setP1(armSub.EncodeAMP());
    } else
    if (svn.getAsBoolean()) {
      armSub.setP2(armSub.EncodeAMP());
    }else
    if (eit.getAsBoolean()) {
      armSub.setP3(armSub.EncodeAMP());
    
    } else { 
      armSub.StopArm();
    }
    
        SmartDashboard.putNumber("GTE2",armSub.EncodeAMP());

    armSub.elevatorMove(two.getAsBoolean(), three.getAsBoolean());
    armSub.Test(four.getAsBoolean(), five.getAsBoolean());
    
SmartDashboard.putNumber("AAEE",armSub.GE());
    /* TEST */
    if (bButton.getAsBoolean()) {
      armSub.AutoAim(/*armSub.IsInBounds()*/);
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
