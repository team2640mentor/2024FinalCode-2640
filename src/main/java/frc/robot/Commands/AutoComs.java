// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.ArmSub;
import frc.robot.Subsystems.IndexSub;
import frc.robot.Subsystems.ShootaSub;
import frc.robot.Subsystems.SuckySub;

public class AutoComs extends Command {
  public static ArmSub armSub;// = new ArmSub();
  public static ShootaSub shootaSub;// = new ShootaSub();
  public static IndexSub indexSub;// = new IndexSub();
  public static SuckySub suckySub;// = new SuckySub();
  /** Creates a new AutoComs. */
  public AutoComs(ArmSub as, ShootaSub ss, IndexSub is, SuckySub sks) {
    armSub =as;
    shootaSub = ss;
    indexSub = is;
    suckySub = sks;
    addRequirements(armSub);
        addRequirements(shootaSub);
            addRequirements(indexSub);
    addRequirements(suckySub);


    // Use addRequirements() here to declare subsystem dependencies.
  }

public static Command armAim(){
    return Commands.run(()->{
      armSub.AutoAim();
    },armSub);
    
}

public static Command armSet(){
    return Commands.run(()->{
      armSub.setP2(armSub.EncodeAMP());
    }, armSub);
}

public static Command feed(){
    return Commands.run(()->{
      indexSub.indexSucky(true,false,false,false,false,false);
    }, indexSub).until(indexSub::SensorCond);
}
public static Command feedShoot(){
  return Commands.run(()->{
    indexSub.indexSucky(false, false, false, true, false, false);
  }, indexSub).withTimeout(.4);
}

public static Command feedStop(){
    return Commands.run(()->{
      indexSub.AutoStop();
    }, indexSub).withTimeout(.1);
}

public static Command intake(){
    return Commands.run(()->{
      suckySub.sucky(true, false);
    }, suckySub)
    .until(indexSub::SensorCond);
}

public static Command intakeStop(){
    return Commands.run(()->{
      suckySub.suckyStop();
    },suckySub).withTimeout(.1);
}

public static Command shootGo(){
  
   return Commands.run(()->{
      shootaSub.shoota(true);
    }, shootaSub).withTimeout(.8);
}

public static Command shootStop(){
    return Commands.run(()->{
      shootaSub.ShootStop();
    },shootaSub).withTimeout(.1);
}
  
}
