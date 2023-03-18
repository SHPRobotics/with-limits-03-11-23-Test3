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
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private  final CANSparkMax armLiftMotor = new CANSparkMax(Constants.ArmConstants.kArmLiftID, MotorType.kBrushless);
  private final RelativeEncoder m_armEncoder = armLiftMotor.getEncoder();

  public  ArmSubsystem() {
//    m_armEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderTick2Degs);
    // set brake mode
    armLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    disableArmSoftLimit();

    //set softLimits
    armLiftMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kArmLimitMin);
    armLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kArmLimitMax);
    enableArmSoftLimit();
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder Value ", m_armEncoder.getPosition());
  }

  public void setMotor(double speed) {
    armLiftMotor.set(speed);
  }

  public void resetArmEncoder(){
    m_armEncoder.setPosition(0);
  }


  public void enableArmSoftLimit(){
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kForward , true);
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse , true);
  } 

  public void disableArmSoftLimit(){
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kForward , false);
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse , false);
  } 

  public RelativeEncoder getArmEncoder(){
    return m_armEncoder;
  }

  public CommandBase armUpMax(){
    return runOnce( ()-> m_armEncoder.setPosition(ArmConstants.kArmLimitMax));
  }

  public CommandBase armDownMax(){
    return runOnce( ()-> m_armEncoder.setPosition(ArmConstants.kArmLimitMin));
  }
  
  public Double getPosition(){
    return m_armEncoder.getPosition();
  }
}
