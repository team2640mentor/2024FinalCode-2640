package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.ArmSub;
// import frc.robot.Subsystems.DriveSub;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.IndexSub;

public class Constants {
    public class Ports{
        //DT
        //public static void WaitCommand(double x){}
        public static IndexSub indexSub = new IndexSub();
        // public static DriveSub driveSub = new DriveSub();
        // public static ArmSub armSub = new ArmSub();

        public static final int leftBack =2;
        public static final int  leftFront = 3;
        public static final int  rightBack =4;
        public static final int  rightFront =5;
        public static final int  intake = 6;
        public static final int  leftArm = 7;
        public static final int  rightArm = 8;
        public static final int  climber = 9;
        public static final int  bottomShoot = 10;
        public static final int  topShoot = 14;
        public static final int  elevator = 12;
        public static final int  mainIndex =13;
        public static final int  subIndex = 11;
    
        public static boolean LED = false; 
        public static final HashMap<String,  Command> AUTO_EVENT_MAP = new HashMap<>();
        //Shoota
        // public static final int topShoota
        // public static final int bottomShoota
        //Index
        //public static final int indexBack= 
        //public static final int indexFront=
        //Sucky
        public static final int sucky = 6;
    }
    public static class EncoderConstants {
    // public static final double ksVolts = 0.096966;
    //public static final double ksVoltSecondsPerMeter = 3.0383;
    //public static final double kaVoltSecondSquaredPerMeter = 0.3163;
    //public static final double kPDriveVel = 0.15318;
        
        //public static final double kTrackWidthMeters = Units.inchesToMeters(0)
        //distance between two wheels horizontally
       // public static final double kMaxSpeedMetersPerSecond = 1;
        //public static final double kMaxAccelerationMetersPerSecondSquared = 1;
      
        public static final double kGearRatio = (30.0/68.0);
        public static final double kWheelRadiusInches = 3;
      //68 rotations= 1 wheel rotation
        public static final double kLinearDistanceConversionFactor =
         (Units.inchesToMeters((2*Math.PI*(kWheelRadiusInches)/(kGearRatio))));
      //public static final double kTrackWidthMeters = Units.inchesToMeters();
      //public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics();
      }
      public class ArmEncoderConstants {
        public static final double AErpm = ((5676*.1)/36)/60;
         public static final double kGearRatio = ((36*22)/72);
        public static final double kWheelRadiusInches = 5.75;
      
        public static final double kLinearDistanceConversionFactor =
         (Units.inchesToMeters(1/(kGearRatio*2*Math.PI*Units.inchesToMeters(kWheelRadiusInches))*10));
      }
    
}
