// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmExtenderConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GripConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPositionCmd;
import frc.robot.commands.Autos;
import frc.robot.commands.BalanceOnBeamCmd;
import frc.robot.commands.MecanumDriveCmd;
import frc.robot.commands.MoveArmCmd;
import frc.robot.commands.MoveArmExtenderCmd;
import frc.robot.commands.ScoreandMove;
import frc.robot.subsystems.ArmExtenderSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ArmExtenderSubsystem m_ArmExtenderSubsystem = new ArmExtenderSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final GripperSubsystem m_GripperSubsystem = new GripperSubsystem();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_JoyLeft = new CommandJoystick(OperatorConstants.kLeftJoystickPort);
  private final CommandJoystick m_JoyRight = new CommandJoystick(OperatorConstants.kRightJoystickPort);
  private final CommandXboxController m_xboxController = new CommandXboxController(OperatorConstants.kXboxControllerPort);

      //private final CommandJoystick m_GamePad = 
  //    new CommandJoystick(OperatorConstants.kGamePadPort);
  //private final CommandXboxController m_XboxController =
    //  new CommandXboxController(OperatorConstants.kXboxControllerPort);

private final Joystick m_leftJoystick = new Joystick(0);
private final Joystick m_rightJoystick = new Joystick(1);
private final Joystick m_gamePad = new Joystick(2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // set defaultCommand
    m_DriveSubsystem.setDefaultCommand(
      new MecanumDriveCmd(m_DriveSubsystem,
                          ()-> -m_JoyLeft.getY(), 
                          ()-> m_JoyLeft.getX(), 
                          ()-> m_JoyRight.getX()));

    m_ArmExtenderSubsystem.setDefaultCommand(
      new MoveArmExtenderCmd(m_ArmExtenderSubsystem, ()-> m_xboxController.getRightY()* Constants.ArmExtenderConstants.kExtenderSpeed)
    );

    m_ArmSubsystem.setDefaultCommand(
      new MoveArmCmd(m_ArmSubsystem, ()-> m_xboxController.getLeftY() * Constants.ArmConstants.kArmSpeed)
    );
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("straight", loadPathplannerTrajectory("deploy/deploy/pathplanner/generatedJSON/straight.wpilib.json", true));
    m_chooser.addOption("curvy", loadPathplannerTrajectory("deploy/deploy/pathplanner/generatedJSON/curvy.wpilib.json", true));

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
                          
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   // new Trigger(m_exampleSubsystem::exampleCondition)
       // .onTrue(new MecanumDriveCmd(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
/*
    // Turn to 90 degrees when button 1 is pressed, with a 5 sec. timeout
    new JoystickButton(m_leftJoystick, OperatorConstants.kJoystickTurnTo90)
      .onTrue(new TurnToAngleCmd(90, m_DriveSubsystem)
      .withTimeout(1.0));

    // Press Button kJoystickGyroReset will reset the Gyro
    new JoystickButton(m_leftJoystick, OperatorConstants.kJoystickGyroReset)
        .whileTrue(new InstantCommand( ()-> m_DriveSubsystem.resetGyro()));

        // BALANCE THE ROBOT USING JOYSTICK --------------------------------------------------------
    // Press Button kJoystickBalanceRobot will balance the robot
    new JoystickButton(m_leftJoystick, OperatorConstants.kJoystickBalanceRobot)
      .onTrue(new AutoBalanceCmd(m_DriveSubsystem.getGyro(),
                                m_DriveSubsystem, 
                                ()-> -m_leftJoystick.getY(),
                                ()-> -m_leftJoystick.getX(), 
                                ()-> -m_rightJoystick.getX()));
*/
    // button 1 to reset gyro
    //new JoystickButton(m_leftJoystick, 1)
    //  .onTrue(new InstantCommand(()-> m_DriveSubsystem.resetGyro()));

   // m_JoyLeft.button(1)
   //   .onTrue(new InstantCommand(()-> m_DriveSubsystem.resetGyro()));

   //Button 1 on Left Joystick to strafe left
  //  m_JoyLeft.button(1)
  //      .onTrue(new MecanumDriveCmd(m_DriveSubsystem, ()->0.0, ()->-0.5,()-> 0.0));
   //Button 1 on right joystick to strafe right
  //  m_JoyRight.button(1)
  //      .onTrue(new MecanumDriveCmd(m_DriveSubsystem, ()->0.0, ()->0.5,()-> 0.0));
        
/*
    new JoystickButton(m_leftJoystick, 2)
      .onTrue(new DriveDistanceCmd(Units.feetToMeters(5.0), m_DriveSubsystem));

      new JoystickButton(m_leftJoystick, 3)
      .onTrue(new DriveDistanceCmd(Units.feetToMeters(-5.0), m_DriveSubsystem));
*/

/*      new JoystickButton(m_rightJoystick, 1)
      .onTrue(new InstantCommand(()-> m_ArmSubsystem.resetArmEncoder()));
      new JoystickButton(m_rightJoystick, 2)
      .onTrue(new InstantCommand(()-> m_ArmSubsystem.disableArmSoftLimit()));
 */
      new JoystickButton(m_rightJoystick, 3)
      .onTrue(new InstantCommand(()-> m_ArmSubsystem.enableArmSoftLimit()));

      new JoystickButton(m_rightJoystick, 4)
      .onTrue(new InstantCommand(()-> m_ArmExtenderSubsystem.resetArmExtender()));
      new JoystickButton(m_rightJoystick, 5)
      .onTrue(new InstantCommand(()-> m_ArmExtenderSubsystem.disableArmExtenderSoftLimits()));
      new JoystickButton(m_rightJoystick, 6)
      .onTrue(new InstantCommand(()-> m_ArmExtenderSubsystem.enableArmExtenderSoftLimits()));

      new JoystickButton(m_rightJoystick, 7)
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.resetGripEncoder()));
      new JoystickButton(m_rightJoystick, 8)
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.disableGripperSoftLimits()));
      new JoystickButton(m_rightJoystick, 9)
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.enableGripperSoftLimits()));


    

/*
    new JoystickButton(m_leftJoystick, 8)
      .onTrue(new TurnToAngleManualPIDCmd(90, m_DriveSubsystem));

    new JoystickButton(m_leftJoystick, 9)
      .onTrue(new TurnToAngleManualPIDCmd(-90, m_DriveSubsystem));
      */
/*
    new JoystickButton(m_leftJoystick, 10)
      .onTrue(new AutoBalanceCmd( m_DriveSubsystem, 
                                  ()-> -m_leftJoystick.getY(),
                                  ()-> m_leftJoystick.getX(), 
                                  ()-> m_rightJoystick.getX()));
/* */   
    new JoystickButton(m_leftJoystick, 10)
      .onTrue(new BalanceOnBeamCmd(m_DriveSubsystem));
/*
    // button 11 to get pitch
    new JoystickButton(m_leftJoystick, 11)
      .onTrue(new InstantCommand(()-> m_DriveSubsystem.getGyroPitch()));

    // button 12 to reset drive encoder
    new JoystickButton(m_leftJoystick, 12)
      .onTrue(new InstantCommand(()-> m_DriveSubsystem.resetEncoders()));
*/
    // button 2 to arm out
    new JoystickButton(m_gamePad, 2)
      .onTrue(new InstantCommand(()-> m_ArmExtenderSubsystem.setMotor(-ArmExtenderConstants.kExtenderSpeed)))
      .onFalse(new InstantCommand(()-> m_ArmExtenderSubsystem.setMotor(0)));
    
    // button 1 to arm in 
    new JoystickButton(m_gamePad, 1)
      .onTrue(new InstantCommand(()-> m_ArmExtenderSubsystem.setMotor(ArmExtenderConstants.kExtenderSpeed)))
      .onFalse(new InstantCommand(()-> m_ArmExtenderSubsystem.setMotor(0)));

    // button 4 arm UP
    new JoystickButton(m_gamePad, 4)
      .onTrue(new InstantCommand(()-> m_ArmSubsystem.setMotor(-Constants.ArmConstants.kArmSpeed)))
      .onFalse(new InstantCommand(()-> m_ArmSubsystem.setMotor(0)));

      // button 3 arm DOWN
    new JoystickButton(m_gamePad, 3)
      .onTrue(new InstantCommand(()-> m_ArmSubsystem.setMotor(Constants.ArmConstants.kArmSpeed)))
      .onFalse(new InstantCommand(()-> m_ArmSubsystem.setMotor(0)));

    // button 6 to grip the cube
    new JoystickButton(m_gamePad, 6)
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.grabCube()))
      .onFalse(new InstantCommand(()-> m_GripperSubsystem.setMotor(0)));

      // button 8 to ungrip the cube
    new JoystickButton(m_gamePad, 8)
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.releaseCube()))
      .onFalse(new InstantCommand(()-> m_GripperSubsystem.setMotor(0)));

      // button 5 to grip the cone
      new JoystickButton(m_gamePad, 5)
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.grabCone()))
      .onFalse(new InstantCommand(()-> m_GripperSubsystem.setMotor(0)));
      // button 7 to ungrip the cone
      new JoystickButton(m_gamePad, 7)
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.releaseCone()))
      .onFalse(new InstantCommand(()-> m_GripperSubsystem.setMotor(0)));

    // xboxcontroller: hold right trigger will grab the cone
    m_xboxController.rightTrigger()
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.grabCone()))
      .onFalse(new InstantCommand(()-> m_GripperSubsystem.setMotor(0)));

    // xboxcontroller: hold left trigger to release the cone
    m_xboxController.leftTrigger()
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.releaseCone()))
      .onFalse(new InstantCommand(()-> m_GripperSubsystem.setMotor(0)));

    //xboxcontroller: hold right bumper to grab the cube
    m_xboxController.rightBumper()
      .onTrue(new InstantCommand(()-> m_GripperSubsystem.grabCube()))
      .onFalse(new InstantCommand(()-> m_GripperSubsystem.setMotor(0)));

    m_xboxController.b()
      .onTrue(new ArmPositionCmd(m_ArmSubsystem, -25))
      .onFalse(new InstantCommand(()-> m_GripperSubsystem.setMotor(0)));
      
}
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    /*
     * if no auto: 
     * return Commands.none();  // or return new InstantCommand();
     */

     //return m_chooser.getSelected();
    //  return Autos.exampleAuto(m_DriveSubsystem, m_ArmSubsystem, m_ArmExtenderSubsystem, m_GripperSubsystem);
    return new ScoreandMove(m_DriveSubsystem, m_ArmExtenderSubsystem, m_ArmSubsystem, m_GripperSubsystem);
  }


  public Command loadPathplannerTrajectory(String filename, boolean resetOdometry){
    Trajectory trajectory;

    try{
      // get the directory name where the trajectory path is located
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      // build the trajectory from PathweaverJson file
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch(IOException exception){
      DriverStation.reportError("unableto open trajectory file "+filename, exception.getStackTrace());

      System.out.println("Unable to read from file"+filename );

      return new InstantCommand();  // do nothing command
    }

    // Construct command to follow trajectory
    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            trajectory,
            m_DriveSubsystem::getPose,
            DriveConstants.kFeedforward,
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

            // Needed for normalizing wheel speeds
            AutoConstants.kMaxSpeedMetersPerSecond,

            // Velocity PID's
            new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
            new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
            new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
            new PIDController(DriveConstants.kPRearRightVel, 0, 0),
            m_DriveSubsystem::getCurrentWheelSpeeds,
            m_DriveSubsystem::setDriveMotorControllersVolts, // Consumer for the output motor voltages
            m_DriveSubsystem);

    if (resetOdometry){
      // Reset odometry to the starting pose of the trajectory.
      m_DriveSubsystem.resetOdometry(trajectory.getInitialPose());
    }

    // Run path following command, then stop at the end.
    return mecanumControllerCommand.andThen(() -> m_DriveSubsystem.drive(0, 0, 0));

  }


}

