// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LimelightSwerveManager;
import frc.robot.commands.PIDCommandTurnToAngle;
import frc.robot.commands.ShootAngleControlCommand;
// import frc.robot.commands.SwerveDirectionPIDCommand;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.DeviceSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterAnglePIDSubsystem;
import frc.robot.subsystems.SwerveDirectionPIDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final SwerveDirectionPIDSubsystem m_leftFrontDirection = new SwerveDirectionPIDSubsystem(Constants.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN, Constants.LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
  private final SwerveDirectionPIDSubsystem m_leftBackDirection = new SwerveDirectionPIDSubsystem(Constants.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN, Constants.LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN);
  private final SwerveDirectionPIDSubsystem m_rightFrontDirection = new SwerveDirectionPIDSubsystem(Constants.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN, Constants.RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
  private final SwerveDirectionPIDSubsystem m_rightBackDirection = new SwerveDirectionPIDSubsystem(Constants.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN, Constants.RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN);
  private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem(m_leftFrontDirection,m_leftBackDirection, m_rightFrontDirection, m_rightBackDirection );
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final ShooterAnglePIDSubsystem m_AnglePIDSubsystem = new ShooterAnglePIDSubsystem();
  private final DeviceSubsystem m_DeviceSubsystem = new DeviceSubsystem();
  private final LedSubsystem m_LedSubsystem = new LedSubsystem();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController =
      new Joystick(OperatorConstants.kDriverControllerPort);
        private final Joystick m_helperController =
      new Joystick(OperatorConstants.kHelperControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("Shoot", new InstantCommand(() -> {
      System.out.println(m_limelight.getAngleToSpeaker());
    }));
    configureBindings();
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
  //  new Trigger(m_exampleSubsystem::exampleCondition)
   //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  //  m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  // Swerve Drive  
  SwerveTeleopCommand DriveMode = new SwerveTeleopCommand(
        this.m_swerveDrive, m_driverController::getY, m_driverController::getX,
        m_driverController::getTwist);
    this.m_swerveDrive.setDefaultCommand(DriveMode);
  // Manually Controlled Shoot Angle
     ShootAngleControlCommand AngleControl = new ShootAngleControlCommand(
         m_helperController::getY,this.m_AnglePIDSubsystem);
    this.m_AnglePIDSubsystem.setDefaultCommand(AngleControl);
    // Limelight Swerve Manager 
    LimelightSwerveManager LimelightSwerveManager = new LimelightSwerveManager(m_limelight, m_swerveDrive);
    // Trigger prints limelight
    new JoystickButton(m_driverController, 1)
    .onTrue(
      new PIDCommandTurnToAngle(m_limelight, m_swerveDrive, LimelightSwerveManager)
    
    //new InstantCommand(() -> {
      //System.out.println("DISTANCE: " + m_limelight.getDistance());
      //System.out.println("ANGLE: " + m_limelight.getAngleToSpeaker());
      // double color = Math.random();
      // m_LedSubsystem.set_led_color(color);
      // System.out.println(color);
    //})
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_swerveDrive, m_DeviceSubsystem);
  }
}
