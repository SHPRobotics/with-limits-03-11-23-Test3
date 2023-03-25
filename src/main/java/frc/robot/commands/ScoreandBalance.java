// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtenderSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreandBalance extends SequentialCommandGroup {
  /** Creates a new ScoreandBalance. */
  public ScoreandBalance(DriveSubsystem drive, ArmExtenderSubsystem extend, ArmSubsystem arm, GripperSubsystem grip) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      
    addCommands(
      /*
      new ArmUpMax(arm),
      new ArmExtenderOutMax(extend),
      new DriveDistanceCmd(0.2, drive, 1.0),
      new ReleaseConeCmd(grip),
      new GrabCubeCmd(grip),
      new ArmExtenderInMax(extend),
      new DriveDistanceCmd(-0.2, drive),
      new ArmDownMax(arm),
       */
      new DriveDistanceCmd(-1, drive, 0.5) , // 2.5 is actually 3.5m unkown why wrong.
      new BalanceOnBeamCmd(drive)
    );
  }
}
