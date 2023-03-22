// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripConstants;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
  private  final CANSparkMax armGripMotor = new CANSparkMax(GripConstants.kArmGripID, MotorType.kBrushless);
    // Grip encoder
  public final RelativeEncoder m_gripEncoder = armGripMotor.getEncoder();
  public DigitalInput m_limitSwitchOpen = new DigitalInput(0);
  public DigitalInput m_limitSwitchClose = new DigitalInput(9);

  public boolean isSetLimitsCube = false;
  public boolean isSetLimitsCone = false;

  public GripperSubsystem() {
    setSoftLimitsCube();
  }    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("GripEncoder value ", m_gripEncoder.getPosition());
    SmartDashboard.putBoolean("LimitSwitch Open", m_limitSwitchOpen.get());
    SmartDashboard.putBoolean("LimitSwitch Close", m_limitSwitchClose.get());
  }

  public void setMotor(double speed) {
    armGripMotor.set(speed);
  }

  public void resetGripEncoder(){
    m_gripEncoder.setPosition(0);
   }

  public void grabCube(){
    if (!isSetLimitsCube) setSoftLimitsCube();

    //if (m_limitSwitchOpen.get() || m_limitSwitchClose.get()) setMotor(0);
    //else
      setMotor(GripConstants.kGripSpeed);
  }

  public void releaseCube(){
    if (!m_limitSwitchOpen.get()) setMotor(0.0);
    else
      setMotor(-GripConstants.kGripSpeed);
  }

  public void grabCone(){
    if (!isSetLimitsCone) setSoftLimitsCone();

    //if (m_limitSwitchOpen.get() || m_limitSwitchClose.get()) setMotor(0);
    //else
      setMotor(GripConstants.kGripSpeed);
  }

  public void releaseCone(){
    if (!m_limitSwitchOpen.get()) setMotor(0.0);
    else
      setMotor(-GripConstants.kGripSpeed);
  }

  public void setSoftLimitsCone(){
    disableGripperSoftLimits(); 

    //set softLimits cone
    armGripMotor.setSoftLimit(SoftLimitDirection.kForward, GripConstants.kGripperConeMin); // 260.0f); //minimum val for cube to be squished old: 125.0
   // armGripMotor.setSoftLimit(SoftLimitDirection.kReverse, -85.0f); // max val for gripper to  be extended old: -85.0
    
    enableGripperSoftLimits();

    isSetLimitsCone = true;
    isSetLimitsCube = false;
  }

  public void setSoftLimitsCube(){
    // if gripper is to the left of Cube's min limit, bring the gripper at 0 position before setting the softLimits
    ///if (m_gripEncoder.getPosition() > 0){ 
    //  m_gripEncoder.setPosition(0) ;
      //Timer.delay(1.0);
    //}

    disableGripperSoftLimits(); 

    //set softLimits for the cube
    armGripMotor.setSoftLimit(SoftLimitDirection.kForward, GripConstants.kGripperLimitMin);   // 147.0f); //minimum val for cube to be squished
    armGripMotor.setSoftLimit(SoftLimitDirection.kReverse, -147.0f); // max val for gripper to  be extended
    
    enableGripperSoftLimits();

    isSetLimitsCube = true;
    isSetLimitsCone = false;

  }

  public void disableGripperSoftLimits(){  
    armGripMotor.enableSoftLimit(SoftLimitDirection.kForward , false);
    armGripMotor.enableSoftLimit(SoftLimitDirection.kReverse , false);
  }

  public void enableGripperSoftLimits(){  
    armGripMotor.enableSoftLimit(SoftLimitDirection.kForward , true);
    armGripMotor.enableSoftLimit(SoftLimitDirection.kReverse , true);
  }

  public RelativeEncoder getGripperEncoder(){
    return m_gripEncoder;
  }

  public DigitalInput getLimitSwitchOpen(){
    return m_limitSwitchOpen;
  }
  
} // End of public class GripperSubsystem
