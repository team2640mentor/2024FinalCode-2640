// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.AnalogInput;
//|CONTROLER STUF|
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ArmCom;
import frc.robot.Commands.AutoComs;
import frc.robot.Commands.AutoDrive;
import frc.robot.Commands.AutoShoot;
import frc.robot.Commands.ClimberCom;
import frc.robot.Commands.DriveCom;
import frc.robot.Commands.IndexCom;
import frc.robot.Commands.ShootaCom;
import frc.robot.Commands.SuckyCom;
import frc.robot.Subsystems.ArmSub;
import frc.robot.Subsystems.ClimberSub;
import frc.robot.Subsystems.DriveSub;
import frc.robot.Subsystems.IndexSub;
import frc.robot.Subsystems.ShootaSub;
import frc.robot.Subsystems.SuckySub;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  private DriveSub driveSub = new DriveSub();
  private IndexSub indexSub = new IndexSub();
  private ShootaSub shootaSub = new ShootaSub();
  private ArmSub armSub = new ArmSub();
  private SuckySub suckySub = new SuckySub();
  private ClimberSub climberSub = new ClimberSub();
 private Joystick joystick = new Joystick(2);
  private XboxController xboxController = new XboxController(0);


  public RobotContainer() {

AutoComs autocoms = new AutoComs(armSub, shootaSub, indexSub, suckySub);
    NamedCommands.registerCommand("shooterGo", AutoComs.shootGo());
    NamedCommands.registerCommand("shoot", AutoComs.feed());
    NamedCommands.registerCommand("feedShoot", AutoComs.feedShoot());
    NamedCommands.registerCommand("shooterStop", AutoComs.shootStop());
    NamedCommands.registerCommand("indexStop", AutoComs.feedStop());
    NamedCommands.registerCommand("armAim", AutoComs.armAim());
    NamedCommands.registerCommand("intakeGo", AutoComs.intake());
    //NamedCommands.registerCommand("intakeStop", AutoComs.intakeStop());
        autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser.addOption("4NoteAutoBL", new AutoComs(armSub, shootaSub, indexSub, suckySub));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();

  }

  private void configureBindings() {
//have you ever been smacked by a wet spaghetti noodle by your girlfriend becasue she had a twin sister, and you accidently screwed her dad?
//Thats how it feels to be a programmer for FIRST robotics?
//>:D
    driveSub.setDefaultCommand(new DriveCom(driveSub,
   ()-> xboxController.getRawAxis(1),
   ()-> xboxController.getRawAxis(4), 
   ()-> xboxController.getRawButton(2), 
   ()-> shootaSub.shootaOn(), 
   ()->indexSub.indexOn(), 
   ()->armSub.ArmOn(), 
   ()->xboxController.getAButton()));
    //()get.yomamas_house;
   
     armSub.setDefaultCommand(new ArmCom(armSub,
     //hi hi :)
    ()-> xboxController.getRawButton(3),
    ()-> joystick.getRawButton(9),
    ()-> joystick.getRawButton(11),
    ()-> joystick.getRawButton(3),
    ()-> joystick.getRawButton(5),
    ()-> joystick.getRawButton(7),
    ()-> joystick.getRawButton(6),
    ()-> joystick.getRawButton(4)));

    shootaSub.setDefaultCommand(new ShootaCom(shootaSub,
     ()-> joystick.getRawButton(2)));


    suckySub.setDefaultCommand(new SuckyCom(suckySub,
    ()-> xboxController.getLeftBumper(),
    ()-> xboxController.getRightBumper()));

    indexSub.setDefaultCommand( new IndexCom(indexSub, 
    ()->joystick.getRawButton(1),  
    ()-> xboxController.getLeftBumper(),
    ()-> xboxController.getRightBumper(), 
    ()-> xboxController.getRawButton(4),
    ()-> indexSub.SensorCond(),
    ()-> joystick.getRawButton(8),
    ()-> joystick.getRawButton(7),
    ()-> joystick.getRawButton(2)));

    climberSub.setDefaultCommand(new ClimberCom(climberSub, 
    ()-> joystick.getRawButton(10), 
    ()->joystick.getRawButton(12)));
  }

 
//howdy
  public Command getAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("Anything");
        //List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Example Auto");

// You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
//Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("Example Auto");

    // Command autoDrive = new StartEndCommand(() -> driveSub.AutoDrive(), () -> driveSub.stopDrive(), driveSub).until(() -> driveSub.getLeftEncoderPos() > -240);
    //AutoIndex autoIndex = new AutoIndex(indexSub); 
    // AutoShoot autoShoot = new AutoShoot(shootaSub);
    // AutoDrive autoDrive = new AutoDrive(shootaSub, driveSub, indexSub);
  // while(driveSub.onNotTarget()){
  //  driveSub.AptMove(driveSub.AptXDistance());
  // }
    // shootaSub.AutoShoot();

    // indexSub.AutoShoot();
    // shootaSub.ShootStop();
    // driveSub.AutoDrive();

  // SequentialCommandGroup auto = new SequentialCommandGroup(autoDrive);
//return SequentialCommandGroup auto = new SequentialCommandGroup(
  //autoDrive, autoIndex,autoShoot);
     return autoChooser.getSelected();
      


   //return new SequentialCommandGroup(new AutoDrive(shootaSub,driveSub, indexSub));

    //return Commands.print("No autonomous command configured");
  }
}
