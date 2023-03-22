// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripConstants;
import frc.robot.subsystems.GripperSubsystem;

public class GrabCubeCmd extends CommandBase {
    GripperSubsystem m_subsystem;
  
    /** Creates a new GrabConeCmd. */
    public GrabCubeCmd(GripperSubsystem subsystem) {
      m_subsystem = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if (!m_subsystem.isSetLimitsCube) m_subsystem.setSoftLimitsCube();
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_subsystem.setMotor(GripConstants.kGripSpeed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_subsystem.setMotor(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return Math.abs(m_subsystem.getGripperEncoder().getPosition()) >= Math.abs(GripConstants.kGripperLimitMin);
    }
  
  }
  