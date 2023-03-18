// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  // OperatorConstants ========================================================================================================================
  public static class OperatorConstants {
    public static final int kLeftJoystickPort = 0;
    public static final int  kRightJoystickPort =1;
    public static final int kXboxControllerPort = 2;
    public static final double kDeadband = 0.05;

    public static final int kOffBalanceAngleThresholdDegrees = 3;
    public static final int kOnBalanceAngleThresholdDegrees = 1;

    public static final int kJoystickTurnTo90 = 1;
    public static final int kJoystickGyroReset = 2;
    public static final int kJoystickBalanceRobot = 3;
    public static final int kGamePadPort = 2;
    
  }

  // DriveConstants ===========================================================================================================================
  public static class DriveConstants{
    // Drive's Motors DeviceID
    public static final int kfrontLeftMotorDeviceID = 1;
    public static final int krearLeftMotorDeviceID  =2;
    public static final int kfrontRightMotorDeviceID =3;
    public static final int krearRightMotorDeviceID =4;

    // Max. speed the robot can drive
    public static final double kMaxOutput = 1.00; //0.75;

    // Conversion factor from tick (pulse) to meter
    public static final double kWheelDiameterMeters = 0.152;     // 6" (or 0.152 meter) diameter of robot's wheel
    public static final int kEncoderCPR = 42;                    // Encoder Counts Per Revolution (https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf)
    public static final double kEncoderGearRatio = 4.16;         // 4.16;
    // kEncoderCPR x 1 tick = 1 revolution = (kWheelDiameterMeters x π) x GearRatio
    //           (kWheelDiameterInches x π)                
    // 1 tick = ──────────────────────────── x GearRatio 
    //                  kEncoderCPR                        
    //
    public static final double kEncoderDistancePerPulseMeters =
        (kWheelDiameterMeters * Math.PI) / kEncoderCPR * kEncoderGearRatio;

    public static final boolean kGyroReversed = false;

    public static final double kTurnP = 0.5;
    public static final double kTurnI = 0.0;    //0.5;
    public static final double kTurnD = 0;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10;

    public static final double kDistP = 0.5;    
    public static final double kDistI = 0.0;
    public static final double kDistD = 0.0;

    // Drive's Kinematics (used in DriveSubsystem)
    public static final double kWheelBase = 0.521;                // 20.5" or 0.521 m Distance in meter between centers of front and back wheels on robot
    public static final double kTrackWidth = 0.686;               // 27" or 0.686 m Distance in meter between centers of right and left wheels on robot
    public static final MecanumDriveKinematics kDriveKinematics = 
        new MecanumDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),     // FL
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),    // FR
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),    // RL
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));  // RR

    // obtain these values from SysId
    public static final SimpleMotorFeedforward kFeedforward = 
      new SimpleMotorFeedforward(0.13647, 2.7713, 0.22908);

    // obtain these values from SysId
    public static final double kPFrontLeftVel = 0.73198;
    public static final double kPRearLeftVel = 0.73198;
    public static final double kPFrontRightVel = 0.73198;
    public static final double kPRearRightVel = 0.73198;
    
   // public static final double kArmTick2Deg = 0.9868;

  }

  // ArmConstants ============================================================================================================================
  public static class ArmConstants{
    public static final int kArmLiftID = 5;
    // test speeds of all arm motors
    public static final double kArmSpeed = 0.4;
    public static final double kArmEncoderTick2Degs = 360/42 * 1.0;
    public static final float kArmRotationForwardLimit = (float) (-50.54); 
    public static final float kArmRotationReverseLimit = 0.0f;

    public static final float kArmLimitMin = 0.0f;
    public static final float kArmLimitMax = -55.0f;
  }

  // Arm Extender ============================================================================================================================
  public static class ArmExtenderConstants{
    public static final int kArmExtenderID = 10;
    public static final double kExtenderSpeed = 0.8;

    public static final float kArmExtenderLimitMin = 0.0f;
    public static final float kArmExtenderLimitMax = -50.0f;

  }

  // GripConstants ============================================================================================================================
  public static class GripConstants{
    public static final int kArmGripID = 7;
    public static final double kGripSpeed = 0.5;
    //public static final double kGripEncoderTick2Meters = Math.PI*Units.inchesToMeters(4.0)/42.0;

    public static final float kGripperLimitMin = 116.0f; //150.0f;
    public static final float kGripperConeMin = 215.0f; //200f;
    public static final float kGripperLimitMax = 50.0f;

  }

  // AutoConstants ============================================================================================================================
  public static class AutoConstants{

    public static final double kautoSpeed = 0.5;
    // The following PID Controller coefficients will need to be tuned for your drive system.
    public static final double ksVolts = 0.13647;
    public static final double kvVoltSecondsPerMeter = 2.7713;
    public static final double kaVoltSecondsPerMeter = 0.22908;
    public static final double kPDriveVel = 0.73198;
    // horizontal distance between the left and right wheels in meters
    //public final double kTrackwidthMeters = Units.inchesToMeters(27);

    //public static final double kMaxSpeedMetersPerSecond = 3.0;
    //public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    //public static final double kRamseteB = 2;
    //public static final double kRamseteZeta = 0.7;
    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;
    public static final double kMaxSpeedMetersPerSecond = 3;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    //public static final double kMaxAngularSpeedRadiansPerSecond = 
    //  DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;

    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    //public static final double kMaxAngularSpeedRadiansPerSecondSquared = 3.0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.35;
    public static final double BEAM_BALANACED_DRIVE_KP = 0.01; //0.015;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;

    public static final double kAutoDriveSpeed = 0.5;
    public static final double kAutoBackupDistanceMeters = Units.inchesToMeters(142.88);
  }
}
