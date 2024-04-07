// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.Ports;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;

public class DriveSub extends SubsystemBase {
  
  private CANSparkMax leftBack = new CANSparkMax(Ports.leftBack, MotorType.kBrushless);
  private CANSparkMax leftFront = new CANSparkMax(Ports.leftFront, MotorType.kBrushless);
  private CANSparkMax rightBack = new CANSparkMax(Ports.rightBack, MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(Ports.rightFront, MotorType.kBrushless);
 double xDis;
 private AHRS navX = new AHRS(SPI.Port.kMXP);
 //private final DifferentialDriveOdometry m_odometry;
  private RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
  private RelativeEncoder rightFrontEncoder = rightFront.getEncoder();
  
  double LFE = leftFrontEncoder.getPosition();
  double RFE;
  boolean isReady = false;
  public DifferentialDrive dDrive;
  Pose2d pose;
   //final static Pose2d t1 = AutoBuilder.getStaringPoseFromAutoFile("ShootOnlyBMID");

  Rotation2d rotation = new Rotation2d(119.2); //red119.2 blue 60.52
    DifferentialDriveKinematics kinematics =
  new DifferentialDriveKinematics(Units.inchesToMeters(21.0));
  private final DifferentialDriveOdometry odometry;
private final DifferentialDrivePoseEstimator poseEstimation;


// AutoBuilder.configureRamsete(this::getPose);
DifferentialDriveWheelSpeeds wheelSpeeds;

private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
 



  
  /* Creates a new DriveSub. */
public DriveSub() {
  
  //SmartDashboard.putString("Iran", "Unwesternized");
 pose = new Pose2d(15.8,6.73,rotation); //red 15.8, 6.73   /blue .79, 6.71 
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    poseEstimation = new DifferentialDrivePoseEstimator(kinematics, angle(), getLeftEncoderPos(), getRightEncoderPos(), setInitPose2d());
    odometry = new DifferentialDriveOdometry(angle(), getLeftEncoderPos(), getRightEncoderPos());
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
leftFrontEncoder.setPositionConversionFactor((EncoderConstants.kLinearDistanceConversionFactor)/22);
rightFrontEncoder.setPositionConversionFactor((EncoderConstants.kLinearDistanceConversionFactor)/22);
leftFrontEncoder.setVelocityConversionFactor((EncoderConstants.kLinearDistanceConversionFactor/60)/22);
rightFrontEncoder.setVelocityConversionFactor((EncoderConstants.kLinearDistanceConversionFactor/60)/22);

    leftBack.setInverted(true);
    rightFront.setInverted(true);
   // m_odometry = new DifferentialDriveOdometry(null, null, null);
    dDrive = new DifferentialDrive(leftFront, rightFront);
    AutoBuilder.configureRamsete(
      this::getPose2d, 
      this::resetPose,
       this::getChassisSpeeds,
        this::drive, new ReplanningConfig(false,false),  () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
   //AutoBuilder.configureRamsete(this::getPose2d, this::resetPose, this::getChassisSpeeds, this::drive, null, null, null);
}

public void resetPose(Pose2d pose){
resetEncoders();
 this.pose = pose;
odometry.resetPosition(angle(), getLeftEncoderPos(), getRightEncoderPos(), pose);
}

public ChassisSpeeds getChassisSpeeds(){
   wheelSpeeds = new DifferentialDriveWheelSpeeds(-leftFrontEncoder.getVelocity(), -rightFrontEncoder.getVelocity());
  return chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

}

public void resetEncoders(){
  leftFrontEncoder.setPosition(0.0);
  rightFrontEncoder.setPosition(0.0);
}

public Pose2d getPose2d(){
  poseEstimation.update(angle(), getLeftEncoderPos(), getRightEncoderPos());
return poseEstimation.getEstimatedPosition();
}
public Rotation2d angle() {
  return navX.getRotation2d();
}
public double getRealSpeeds(){
  return rightFront.get();
}
public double getSpeeds(){
   wheelSpeeds = new DifferentialDriveWheelSpeeds(-leftFrontEncoder.getVelocity(), -rightFrontEncoder.getVelocity());

  return wheelSpeeds.rightMetersPerSecond/21.4;

}



public void updateSpeeds()  {
  chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0 ,0, 0, Rotation2d.fromDegrees(0));
}
public void drive(ChassisSpeeds CS){
  wheelSpeeds = kinematics.toWheelSpeeds(CS);
 double LeftSpeed = wheelSpeeds.leftMetersPerSecond/21.4;
 double RightSpeed = wheelSpeeds.rightMetersPerSecond/21.4;
  leftFront.set(LeftSpeed);//CS.vxMetersPerSecond);
  rightFront.set(RightSpeed);//CS.vxMetersPerSecond);
}

public Pose2d setInitPose2d(){
 return pose;//LimelightHelpers.getBotPose2d("limelight-driver");
}

public double AptXDistance() {

    xDis = LimelightHelpers.getTX("limelight-driver");
    return xDis;
}

double h  =64;
double kD = 18;
double x;
double ty;
double tx;



public void encoderReset(){
    leftFrontEncoder.setPosition(0);
}

public void AptMove(double ComDis) {

  //moves back
      ty = LimelightHelpers.getTY("limelight-driver")+22;
      tx = LimelightHelpers.getTX("limelight-driver");

//   if((39/Math.tan(Math.toRadians(ty)))<48 ){

//     ty = LimelightHelpers.getTY("limelight-driver")+22;
//     leftFront.set(-.05);
//     rightFront.set(-.05);

//   }//moves forward
//    else if ((39/Math.tan(Math.toRadians(ty))>54)){

//   leftFront.set(.05);
//   rightFront.set(0.05);

//  }
// //turns right
//  else 
 if (tx < -3) {
      isReady = false;
      leftFront.set(0.05);
      rightFront.set(-0.05);
   }//turn left
     else if (tx > 3) {
      isReady  = false;
      rightFront.set(0.05);
      leftFront.set(-0.05);
  }//Stop on target
   else if((tx > -3)&& (tx <3)){
       isReady = true;
       leftFront.set(0);
      rightFront.set(0);
 }
}
    
  
  public boolean IsReady(){
    return isReady;
  }
  
  public void AutoDrive(){ //0.6366*68=43.2888
    //close note pos=77"
    //  while(leftFrontEncoder.getPosition()>-240){//kill
    leftFront.set(-.1);
    rightFront.set(-.15);
    //  }
  }

  public void AutoStop(){
     leftFront.set(0);
    rightFront.set(0);
  }



  

  public void stopDrive() {
    leftFront.set(0);
    rightFront.set(0);
  }

public boolean onNotTarget(){
        ty = LimelightHelpers.getTY("limelight-driver")+22;

  SmartDashboard.putBoolean("OnTarg", ((AptXDistance() > -3 && AptXDistance() <3))&&((39/Math.tan(Math.toRadians(ty)))>48 && 39/Math.tan(Math.toRadians(ty)) < 54));
    if ((AptXDistance() > -3 && AptXDistance() <3)||((39/Math.tan(Math.toRadians(ty)))>48 && 39/Math.tan(Math.toRadians(ty)) < 54)){
      //SmartDashboard.putString("Iran", "Succsesfully invaded");
            isReady = false;

      return false;
    } else {
      return true;
    }
}

  public double getLeftEncoderPos() {
    LFE = leftFrontEncoder.getPosition();
    return LFE;
  }

  public double getRightEncoderPos() {
    RFE = rightFrontEncoder.getPosition();
    return LFE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
