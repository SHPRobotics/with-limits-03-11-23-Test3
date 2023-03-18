// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtenderSubsystem;

public class MoveArmExtenderCmd extends CommandBase {

  private final ArmExtenderSubsystem m_ArmExtenderSubsystem;
  private final DoubleSupplier m_Speed;
  
  /** Creates a new MoveArmExtenderCmd. */
  public MoveArmExtenderCmd(ArmExtenderSubsystem subsystem, DoubleSupplier speed) {
   
    m_ArmExtenderSubsystem = subsystem;
    m_Speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double realSpeed = m_Speed.getAsDouble();

    m_ArmExtenderSubsystem.setMotor(realSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ArmExtenderSubsystem.setMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
