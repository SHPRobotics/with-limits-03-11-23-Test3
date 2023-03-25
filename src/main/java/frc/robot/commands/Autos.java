// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmExtenderSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(  // Put in Base for no auton
      DriveSubsystem driveSubsystem, 
      ArmSubsystem armSubsystem, 
      ArmExtenderSubsystem armExtenderSubsystem,
      GripperSubsystem gripperSubsystem) {

    return Commands.sequence(
/*    
      //subsystem.exampleMethodCommand(), 
      //new ExampleCommand(subsystem)
     // Commands.parallel(
        // arm up maximum
        armSubsystem.armUpMax(),

        // armExtender out maximum
        armExtenderSubsystem.armExtenderOutMax(),
      

      // release cube
      gripperSubsystem.releaseCube(),

      //Commands.parallel(
        // armExtender in maximum
        armExtenderSubsystem.armExtenderInMax(),

        //armSubsystem.armDownMax(),
*/
        // drive backward a distance to leave COMMUNITY
        new FunctionalCommand(
          //onInit: reset drive encoder
          driveSubsystem::resetEncoders, 
          // onExecute: drive backward
          ()-> driveSubsystem.drive(-AutoConstants.kAutoDriveSpeed, 0.0, 0.0), 
          // onEnd: stopthe robot
          interrupt -> driveSubsystem.drive(0.0, 0.0, 0.0), 
          // isFinished
          ()-> driveSubsystem.getAverageEncoderDistance() <= AutoConstants.kAutoBackupDistanceMeters , 
          driveSubsystem)
      );

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
