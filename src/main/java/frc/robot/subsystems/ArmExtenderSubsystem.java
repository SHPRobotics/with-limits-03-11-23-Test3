// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmExtenderConstants;

public class ArmExtenderSubsystem extends SubsystemBase {
  /** Creates a new ArmExtenderSubsystem. */
  private final CANSparkMax m_armExtenderMotor = new CANSparkMax(ArmExtenderConstants.kArmExtenderID, MotorType.kBrushless);
  private final RelativeEncoder m_armExtenderEncoder = m_armExtenderMotor.getEncoder();

  
  public ArmExtenderSubsystem() {
    // set brake mode
    m_armExtenderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    
    disableArmExtenderSoftLimits();
     //set softLimits
    m_armExtenderMotor.setSoftLimit(SoftLimitDirection.kForward, ArmExtenderConstants.kArmExtenderLimitMin);
    m_armExtenderMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmExtenderConstants.kArmExtenderLimitMax);
    enableArmExtenderSoftLimits();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmExtender Encoder Value ", m_armExtenderEncoder.getPosition());    
  }
  
  public void setMotor(double speed) {
    m_armExtenderMotor.set(speed);
  }

  public void resetArmExtender(){
    m_armExtenderEncoder.setPosition(0);
  }

  public void enableArmExtenderSoftLimits(){
    m_armExtenderMotor.enableSoftLimit(SoftLimitDirection.kForward , true);
    m_armExtenderMotor.enableSoftLimit(SoftLimitDirection.kReverse , true);

  }

  public void disableArmExtenderSoftLimits(){
    m_armExtenderMotor.enableSoftLimit(SoftLimitDirection.kForward , false);
    m_armExtenderMotor.enableSoftLimit(SoftLimitDirection.kReverse , false);
  }

  public RelativeEncoder getArmExtenderEncoder(){
    return m_armExtenderEncoder;
  }

  public CommandBase armExtenderOutMax(){
    return runOnce( ()-> m_armExtenderEncoder.setPosition(ArmExtenderConstants.kArmExtenderLimitMax));
  }

  public CommandBase armExtenderInMax(){
    return runOnce( ()-> m_armExtenderEncoder.setPosition(ArmExtenderConstants.kArmExtenderLimitMin));
  }

} // public class ArmExtenderSubsystem
