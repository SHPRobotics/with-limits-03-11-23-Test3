// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCmd extends CommandBase {
  public final ArmSubsystem m_subsystem;
  public final Double m_position;

  /** Creates a new ArmPositionCmd. */
  public ArmPositionCmd(ArmSubsystem subsystem, double position) {
    
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
    if (Math.abs(m_position) > Math.abs(m_subsystem.getPosition()))
      m_subsystem.setMotor(-AutoConstants.kAutoArmSpeed); //move arm up
    else if (Math.abs(m_position) < Math.abs(m_subsystem.getPosition()))
      m_subsystem.setMotor(AutoConstants.kAutoArmSpeed); //move arm down
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
    
    boolean atPos;
    if (Math.abs(m_subsystem.getPosition()) >= (Math.abs(m_position)-1) && Math.abs(m_subsystem.getPosition()) <= (Math.abs(m_position)+1))
    atPos = true;
    else
      atPos = false;

    return atPos; //returns true when the encoder position is equal to the input position

  }
}
