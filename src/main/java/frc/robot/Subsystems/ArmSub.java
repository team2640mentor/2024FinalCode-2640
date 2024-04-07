// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;

//import frc.robot.Constants.EncoderConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.LimelightHelpers;

public class ArmSub extends SubsystemBase {

  private CANSparkMax rightArm = new CANSparkMax(Ports.rightArm, MotorType.kBrushless);
  private CANSparkMax leftArm = new CANSparkMax(Ports.leftArm, MotorType.kBrushless);
  private CANSparkMax elevator = new CANSparkMax(Ports.elevator, MotorType.kBrushless);

  private AbsoluteEncoder AE = rightArm.getAbsoluteEncoder();
  private RelativeEncoder EE = elevator.getEncoder();
  double LFEP = AE.getPosition();
  double r;
boolean sourceReady = false;

  double targetPos;
  boolean EleRedy = false;
  boolean armOn = false;
  boolean eleOn = false;

public ArmSub() {
    leftArm.setInverted(false);
    rightArm.setInverted(false);

    EE.setPosition(0);

}

public double getAePos() {
    return AE.getPosition();
}

public void reset() {
    armOn = false;
    EE.setPosition(0);

}
//Sets to Middle Position
public void setP1(double AmpTP) {

    double targetPos = .5;
    armOn = true;
    if ((AE.getPosition() > .55) || (AE.getPosition() < .45)) {
      SmartDashboard.putNumber("AE", AE.getPosition());
      if ((AmpTP>.45)&&(AmpTP<.55)){
        sourceReady = true;
      }
      if (AmpTP > 1) {
        r = 1;
      }else{
        r = (AE.getPosition() - targetPos) / Math.abs(AE.getPosition() - targetPos);
      }
      if (((AE.getPosition() - targetPos) * r > .1) || AmpTP>1 ){
        leftArm.set(0.5 * r);
        rightArm.set(-0.5 * r);
    
      } else {
        leftArm.set(0.05 * r);
        rightArm.set(-0.05 * r);

      }
    } else {
      leftArm.set(0);
      rightArm.set(0);
      armOn = false;
    }
}
//Sets to down position
public void setP2(double AmpTP) {

    double targetPos = .088;
    armOn = true;

    if ((AE.getPosition() > 0.085) || (AE.getPosition() < 0.09)) {
      SmartDashboard.putNumber("AE", AE.getPosition());
    if (AmpTP > 1) {
        r = 1;
    }else{
      r = (AE.getPosition() - targetPos) / Math.abs(AE.getPosition() - targetPos);
    }
      if ((AmpTP - targetPos) * r > .1) {
        leftArm.set(0.5 * r);
        rightArm.set(-0.5 * r);

      } else {
        leftArm.set(0.05 * r);
        rightArm.set(-0.05 * r);

      }
    } else {

      leftArm.set(0);
      rightArm.set(0);

      armOn = false;
    }
}
//Sets to AMP position
public void setP3(double AmpTP) {

    double targetPos = 1.028;
    armOn = true;

    if ((AmpTP > 1.0141) || (AmpTP < 1.030)) {
      SmartDashboard.putNumber("AE", AE.getPosition());

      r = (AmpTP - targetPos) / Math.abs(AmpTP - targetPos);
      if (AmpTP > .69) {
        EleRedy = true;
      }
      if ((AmpTP - targetPos) * r > .25) {
        leftArm.set(0.7 * r);
        rightArm.set(-0.7 * r);

      } else if((AmpTP - targetPos)*r> .05){
        leftArm.set(0.35 * r);
        rightArm.set(-0.35 * r);
      }else{
        leftArm.set(0.05 * r);
        rightArm.set(-0.05 * r);
      }
    } else {

      leftArm.set(0);
      rightArm.set(0);

      armOn = false;
    }
}

public void StopArm() {
    sourceReady = false;
    EleRedy = false;
    armOn = false;

    leftArm.set(0);
    rightArm.set(0);
    if(!(AE.getPosition() < EA + .01 || AE.getPosition() > EA - 0.01)){
      SmartDashboard.putBoolean("AutoAim", true);
    }
}

public void elevatorMove(Boolean three, Boolean five) {
  //Elevator retract
    if (three) {
          eleOn = true;

      elevator.set(.20);
   
    } //Elevator extend
    else if (five && (EE.getPosition() > -1200)) {
      if (EE.getPosition() > -1100) {
            eleOn = true;

        elevator.set(-.3);
      } else {
        eleOn = false;
        elevator.set(0);
      }

    }//Elevator Amp Extension 
     else if (EleRedy && (EE.getPosition() > -1200)) {
      if (EE.getPosition() > -1150) {
         eleOn = true;
        elevator.set(-.30);
      } else {
         eleOn = true;
        elevator.set(-0.15);
      }
    }else if (sourceReady) {
      if (EE.getPosition() > -600) {
         eleOn = true;
        elevator.set(-.30);
      } else {
         eleOn = true;
        elevator.set(-0.15);
      }
    }//stops
    else {
      if (!(EleRedy || three || five)) {
        EE.setPosition(0);
      }
      eleOn = false;
      elevator.set(0);
    }
}
//AutoAim variables 
  double h = 72;// 78 real
  double kD = 5;
  double ty;
  double tx;
  double x;
  double EA;
  double ATHeight = 39;
  double r2;
  double EA2;
  double kArmAng = 110;
  double LlAngle = 21;

public void AutoAim(/* Boolean InBounds */) {

    tx = (LimelightHelpers.getTX("limelight-driver"));
    ty = LimelightHelpers.getTY("limelight-driver") + LlAngle;
    x = Math.toDegrees(Math.atan(h / (kD + (ATHeight / Math.tan(Math.toRadians(ty))))));

   
    //Checks if required angle is in Bounds //Simplify later
    if (((x - 55) / 110 > .05) || ((x - 55) / 110) < -.05) {
      EA2 = ((x - 55) / 110);
      r2 = (EA2) / Math.abs(EA2);
      EA = ((x - 55) / 110) * r2;

    //Angles Arm to required postion
    if((AE.getPosition()> EA+.1) || (AE.getPosition() < EA -.1) ){
       r = (EA - AE.getPosition()) / Math.abs(EA - AE.getPosition());
        armOn = true;
        leftArm.set(-0.3 * r);
        rightArm.set(0.3 * r);
              SmartDashboard.putBoolean("AutoAim", false);

    }else
      if (((AE.getPosition() > EA + .01 || AE.getPosition() < EA - 0.01) /* && InBounds */)) {
        r = (EA - AE.getPosition()) / Math.abs(EA - AE.getPosition());
        armOn = true;
        leftArm.set(-0.1 * r);
        rightArm.set(0.1 * r);
              SmartDashboard.putBoolean("AutoAim", false);

      }
    
    }
}

public double GE() {
    return AE.getPosition();
}
  

  double disMove = AE.getPosition();
  double disPH = AE.getPosition();

public double GTE() {

    disPH = disPH + AE.getPosition() - disMove;
    disMove = AE.getPosition();

    return disPH;
}

  double disPH2 = 0;
  Boolean o = false;
  Boolean u = false;
  Boolean OutBound = false;

public double EncodeAMP() {
    if (o && AE.getPosition() < .3) {
      OutBound = true;
    } else {
      OutBound = false;
    }
    if (AE.getPosition() > .5 || OutBound) {
      o = true;
      u = false;
    } else if (!(o && OutBound)) {
      o = false;
      u = true;

    }
    if (o && OutBound) {
      disPH2 = AE.getPosition() + 1;
    }
    if (!OutBound) {

      disPH2 = AE.getPosition();
    }

    return disPH2;
}

public boolean IsInBounds() {
    return (AE.getPosition() > .2 && AE.getPosition() < .9);
}

//Manual Arm Contorl
public void Test(boolean six, boolean four) {
    SmartDashboard.putNumber("AE", AE.getPosition());
    //Arm moevs up
    if (six) {
      armOn = true;
      rightArm.set(.5);
      leftArm.set(-0.5);

    } // Arm moves down
    else if (four) {
      armOn = true;
      rightArm.set(-0.5);
      leftArm.set(0.5);


    } // Arm stops
     else if(!armOn) {
      armOn = false;
      rightArm.set(0);
      leftArm.set(0);

    }
}

public boolean ArmOn(){
  return armOn;
}
public boolean EleOn(){
  return eleOn;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
