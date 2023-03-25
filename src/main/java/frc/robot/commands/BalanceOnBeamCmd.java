// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceOnBeamCmd extends CommandBase {
  private double error;
  private double currentAngle;
  private double drivePower;
  private final DriveSubsystem m_DriveSubsystem;
private static int attempt;

  /** Creates a new BalanceOnBeamCmd. */
  public BalanceOnBeamCmd(DriveSubsystem subsystem) {
    m_DriveSubsystem = subsystem;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
//    currentAngle = m_DriveSubsystem.getGyro().getPitch();
    currentAngle = m_DriveSubsystem.getGyroPitch();
//    error = Constants.BEAM_BALANCED_GOAL_DEGREES - currentAngle;
    error = -AutoConstants.BEAM_BALANCED_GOAL_DEGREES + currentAngle;
//    drivePower = -Math.min(Constants.BEAM_BALANACED_DRIVE_KP * error, 1);
//    drivePower = Constants.BEAM_BALANACED_DRIVE_KP * error;
    drivePower = .0115 * error; //.03 //.008 //0115
    SmartDashboard.putNumber("drivePower", drivePower);
    
attempt++;
System.out.print("Run # "+ attempt +
                ", sensor: " + currentAngle +
                ", error: " + error +
                ", drivPower="+ drivePower);

    // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    if (drivePower < 0) {
      drivePower *= AutoConstants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
System.out.print(", boost power: "+ drivePower);      
    }
else
System.out.print(", no-boost power: "+ drivePower);      

    // Limit the max power
    if (Math.abs(drivePower) > 0.4) {
      drivePower = Math.copySign(0.4, drivePower);
System.out.print(", max power: "+ drivePower);
    }
else
System.out.print(", no-max power: "+ drivePower);
System.out.println();

    m_DriveSubsystem.drive(drivePower, 0,0);

    if(Math.abs(error) < AutoConstants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES){
      m_DriveSubsystem.drive(0, 0,0);
    } // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees))
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Math.abs(error) < AutoConstants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    return false;
  }
}
