// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCmd extends CommandBase {
  private final double m_distanceMeters;
  private final DriveSubsystem m_subsystem;

  /** Creates a new DriveDistance. */
  public DriveDistanceCmd(double distanceMeters, DriveSubsystem subsystem) {
    m_distanceMeters = distanceMeters;
    m_subsystem = subsystem;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_distanceMeters >0)
      m_subsystem.drive(AutoConstants.kautoSpeed, 0, 0);  // drive forward
    else
      m_subsystem.drive(-AutoConstants.kautoSpeed, 0, 0); // drive reverse

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getAverageEncoderDistance()) >= Math.abs(m_distanceMeters);
  }
}
