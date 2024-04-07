// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;


public class IndexSub extends SubsystemBase {
  /** Creates a new Indexer. */
    private AnalogInput sensor = new AnalogInput(0);

  private CANSparkMax mainIndexer = new CANSparkMax(Ports.mainIndex, MotorType.kBrushless);
  private CANSparkMax subIndexer = new CANSparkMax(Ports.subIndex, MotorType.kBrushless);
  boolean hasTripped = false;
  boolean indexOn = false;
  RelativeEncoder ME = mainIndexer.getEncoder();

public IndexSub() {
  ME.setPosition(0);
    subIndexer.setInverted(true);
}
public void resetEncoder(){
  ME.setPosition(0);
}



public void indexSucky(boolean LB, boolean RB, boolean sensor, boolean trig, boolean svn, boolean tw ) {

    //checks if sensor has tripepd
    if (!sensor) {
      hasTripped = true;
    }
    //reverses after tripping
    if ((hasTripped && sensor)&&LB) {
      indexOn = true;
      mainIndexer.set(0.3);
      subIndexer.set(0.3);
    } //Intakes until note trips sensor
     else
    if (LB && !hasTripped) {
      indexOn = true;
      mainIndexer.set(-0.8);
      subIndexer.set(-0.8);

    } //Outakes
    else if (RB) {
      resetRB();
      indexOn = true;
      mainIndexer.set(0.4);
      subIndexer.set(0.4);
      
    } //Shoots a note
    else if (trig) {

      resetRB();
      indexOn = true;
      mainIndexer.set(-0.99);
      subIndexer.set(-0.99);


   } else if (svn&&tw){
       mainIndexer.set(-0.99);
       subIndexer.set(-0.99);
     }//Stops
    else {
      indexOn = false;
      mainIndexer.set(0);
      subIndexer.set(0);
    }
}
boolean senny = SensorCond();
public void AutoSuck(){

  while(senny){
 mainIndexer.set(-0.8);
 subIndexer.set(-0.8);
 senny = SensorCond();
  } 
  while(senny){
    senny = SensorCond();
mainIndexer.set(0.3);
subIndexer.set(0.3);
  }

}
public boolean SensorCond(){
  return sensor.getValue() >400;
}

//Stops sensor from tripping when RB is pressed
public void resetRB() {
    hasTripped = false;
}

//Autonomous Shoot
public void AutoShoot() {
    resetRB();
    mainIndexer.set(-0.99);
    subIndexer.set(-0.99);

}

public void AmpShot(double shooterPos) {
   mainIndexer.set(-0.99);
   subIndexer.set(-0.99);
}

public void trapHo(){
  resetRB();
  mainIndexer.set(0.99);
  subIndexer.set(-0.99);
}
public void AutoStop(){
     mainIndexer.set(0);
     subIndexer.set(0);
}

public boolean indexOn(){
  return indexOn;
}
public boolean HasTripped(){
  return hasTripped;
}

public double GE(){
  return ME.getPosition();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

 // Trying to get the wait comand to work

      //
      // WaitCommand(20);
      // Ports.WaitCommand(20);
      // The code below does not work because it is in an interrupt
      // Would have to set this up with a seperate thread
      // Thread.sleep(500);
      // This should work.
      // If moving forward when button is pushed then it will
      // Continue to move.
      // Bot must be stationary.
      // If you want other functionality then you will need Thread
      // When calling the intrupt, put it to its own thread.
      // That way you can sleep that thread.
      // Timer.delay(100);
      //
