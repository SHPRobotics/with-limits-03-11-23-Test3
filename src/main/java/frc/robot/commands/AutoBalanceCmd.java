// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCmd extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final AHRS m_gyro;
  private final DoubleSupplier m_xSpeed;
  private final DoubleSupplier m_ySpeed;
  private final DoubleSupplier m_rotSpeed;

  private  Boolean autoBalanceXMode = false;
  private  Boolean autoBalanceYMode = false;

private static int attempt = 0;

  /** Creates a new GyroAutoBalance. */
  public AutoBalanceCmd( DriveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
    m_gyro = subsystem.getGyro();
    m_subsystem = subsystem;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rotSpeed = rotSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAxisRate = m_xSpeed.getAsDouble();
    double yAxisRate = m_ySpeed.getAsDouble();
    double rotAxisRate = m_rotSpeed.getAsDouble();

    double pitchAngleDegrees = m_gyro.getPitch();
    double rollAngleDegrees = m_gyro.getRoll();

    // find out if robot is off balance in X direction (pitch)
    if (!autoBalanceXMode && (Math.abs(pitchAngleDegrees) >= Math.abs(OperatorConstants.kOffBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = true;
    }
    else if (autoBalanceXMode && (Math.abs(pitchAngleDegrees) <= Math.abs(OperatorConstants.kOnBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = false;
    }

    // find out if robot is off balance in Y direction (roll)
    if (!autoBalanceYMode && (Math.abs(pitchAngleDegrees) >= Math.abs(OperatorConstants.kOffBalanceAngleThresholdDegrees))) {
      autoBalanceYMode = true;
    }
    else if (autoBalanceYMode && (Math.abs(pitchAngleDegrees) <= Math.abs(OperatorConstants.kOnBalanceAngleThresholdDegrees))) {
      autoBalanceYMode = false;
    }
attempt++;
System.out.println("Attempt " + attempt + "autoBalanceXMode= "+ autoBalanceXMode+", autoBalanceYMode= "+autoBalanceYMode);

    // Control drive system automatically,
    // driving in reverse direction of pitch/roll angle,
    // with a magnitude based upon the angle

    if (autoBalanceXMode) {
          double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
          xAxisRate = Math.sin(pitchAngleRadians) * -1;
    }
    if (autoBalanceYMode) {
          double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
          yAxisRate = Math.sin(rollAngleRadians) * -1;
    }
System.out.println("xAxisRate = "+xAxisRate+", yAxisRate = "+ yAxisRate+", rotAxisRate = "+rotAxisRate);
    // now drive in reverse direction
    try {
//      m_subsystem.drive(xAxisRate, yAxisRate, rotAxisRate, new Rotation2d());
      m_subsystem.drive(xAxisRate, 0, 0, new Rotation2d());
    } 
    catch (RuntimeException ex) {
      String err_string = "Drive system error:  " + ex.getMessage();
      DriverStation.reportError(err_string, true);
    }

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!autoBalanceXMode && !autoBalanceYMode);
  }
}
