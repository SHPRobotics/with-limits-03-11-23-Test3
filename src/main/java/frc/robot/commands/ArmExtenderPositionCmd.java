// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmExtenderConstants;
import frc.robot.subsystems.ArmExtenderSubsystem;

public class ArmExtenderPositionCmd extends CommandBase {
  public final ArmExtenderSubsystem m_subsystem;
  public final Double m_position;

  /** Creates a new ArmExtenderPositionCmd. */
  public ArmExtenderPositionCmd(ArmExtenderSubsystem subsystem, Double position) {
    m_subsystem = subsystem;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_position > m_subsystem.getPosition())
      m_subsystem.setMotor(-ArmExtenderConstants.kExtenderSpeed); //Move arm out when input position is greater than enconder position
    else if (m_position < m_subsystem.getPosition())
      m_subsystem.setMotor(ArmExtenderConstants.kExtenderSpeed); //Move arm in when inpurt position is less than enconder position
    else
      m_subsystem.setMotor(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.getPosition() == m_position; //returns true when extender position is equal to the input position
  }
}
