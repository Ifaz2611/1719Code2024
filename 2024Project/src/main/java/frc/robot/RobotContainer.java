// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.LimeLightMovePIDCommand;
import frc.robot.commands.ShootAngleControlCommand;
import frc.robot.commands.ShootSequence;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoMovePIDCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.LimelightSwerveManager;
import frc.robot.commands.PIDCommandTurnToAngle;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DeviceSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterAnglePIDSubsystem;
import frc.robot.subsystems.SwerveDirectionPIDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDirectionPIDSubsystem m_leftFrontDirection = new SwerveDirectionPIDSubsystem(
      Constants.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN, Constants.LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
  private final SwerveDirectionPIDSubsystem m_leftBackDirection = new SwerveDirectionPIDSubsystem(
      Constants.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN, Constants.LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN);
  private final SwerveDirectionPIDSubsystem m_rightFrontDirection = new SwerveDirectionPIDSubsystem(
      Constants.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN, Constants.RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
  private final SwerveDirectionPIDSubsystem m_rightBackDirection = new SwerveDirectionPIDSubsystem(
      Constants.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN, Constants.RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN);
  private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem(m_leftFrontDirection, m_leftBackDirection,
      m_rightFrontDirection, m_rightBackDirection);
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final ShooterAnglePIDSubsystem m_AnglePIDSubsystem = new ShooterAnglePIDSubsystem();
  private final DeviceSubsystem m_DeviceSubsystem = new DeviceSubsystem();
  private final LedSubsystem m_LedSubsystem = new LedSubsystem();
   private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private final Joystick m_helperController = new Joystick(OperatorConstants.kHelperControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   JoystickButton intake = new JoystickButton(m_helperController, 1);
      JoystickButton outake = new JoystickButton(m_helperController, 7);

  private void configureBindings() {

          //             System.out.println("sent inside");

          // double x = 

          // System.out.println(x + " sensor");

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // Swerve Drive
    SwerveTeleopCommand DriveMode = new SwerveTeleopCommand(
        this.m_swerveDrive, m_driverController::getY, m_driverController::getX,
        m_driverController::getTwist);
    this.m_swerveDrive.setDefaultCommand(DriveMode);
    // Manually Controlled Shoot Angle

    ShootAngleControlCommand AngleControl = new ShootAngleControlCommand(
    this.m_AnglePIDSubsystem, m_limelight);


    this.m_AnglePIDSubsystem.setDefaultCommand(AngleControl);

      // new InstantCommand(() -> {
  

      //   });
    // Trigger prints limelight

    // this should activate the intake motors
    intake.onTrue(
        new InstantCommand(() -> {
          m_DeviceSubsystem.turnIntakeMotors(Constants.INTAKESPEED);
        }));

        outake.onTrue(
        new InstantCommand(() -> {
          m_DeviceSubsystem.turnIntakeMotors(Constants.OUTAKESPEED);
        }));

        if (!intake.getAsBoolean() && !outake.getAsBoolean()){
          new InstantCommand(() -> {
          m_DeviceSubsystem.turnOffIntakeMotors();
        });
        }

    // this should activate the shooter motors
    new JoystickButton(m_helperController, 2).onTrue(
        new InstantCommand(() -> {
          // System.out.println("outside");

          m_DeviceSubsystem.turnShooterMotors(1);
        }));
    new JoystickButton(m_helperController, 2).onFalse(
        new InstantCommand(() -> {
          m_DeviceSubsystem.turnShooterMotors(0);
        }));

        new JoystickButton(m_helperController, 8).onTrue(
        new InstantCommand(() -> {
          System.out.println(m_DeviceSubsystem.checkRing());
        }));

    new JoystickButton(m_helperController, 3).onTrue(
       
            new LimelightSwerveManager(m_limelight, m_swerveDrive)
             

            ) ;
    new JoystickButton(m_helperController, 4).onTrue(
            new ShootSequence(m_DeviceSubsystem)
    );

        new JoystickButton(m_helperController, 5).onTrue(
        new InstantCommand(() -> {
          m_ClimbSubsystem.raise();
        }));

        new JoystickButton(m_helperController, 6).onTrue(
        new InstantCommand(() -> {
          m_ClimbSubsystem.lower();
        }));
new JoystickButton(m_driverController, 1).onTrue(

new AutoMovePIDCommand(0,  30,  m_swerveDrive)
);

      

        
    // new Trigger(() -> ((m_helperController.getY() == 1)))
    //     .onTrue(
    //         new LimelightSwerveManager(m_limelight, m_swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String m_autoSelected) {
    // An example command will be run in autonomous
    if (m_autoSelected.equals("Auto 1")) {
      return Autos.Auto1(m_swerveDrive, m_DeviceSubsystem, m_limelight);
    } else if (m_autoSelected.equals("Auto 2")) {
      return Autos.Auto2(m_swerveDrive, m_DeviceSubsystem);
    } else if (m_autoSelected.equals("Auto 3")) {
      return Autos.Auto3(m_swerveDrive, m_DeviceSubsystem);
    } else {
      return Autos.defaultAuto(m_swerveDrive, m_DeviceSubsystem);
    }
  }
}
