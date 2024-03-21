// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.DeviceSubsystem;
import frc.robot.subsystems.ShooterAnglePIDSubsystem;
import frc.robot.subsystems.SwerveDirectionPIDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public static Pigeon2 GYRO = new Pigeon2(Constants.CAN_GYRO_PORT);

  // Robot auton choices
  private final String kAuto0 = "Auto 0 (Default)";
  private final String kAuto1 = "Auto 1";
  private final String kAuto2 = "Auto 2";
  private final String kAuto3 = "Auto 3";
  // private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    CameraServer.startAutomaticCapture();
    UsbCamera USBCAM = CameraServer.startAutomaticCapture();
    USBCAM.setResolution(720, 540);


    // makes a new camera I HOPE :)
    CameraServer.startAutomaticCapture();

    m_robotContainer = new RobotContainer();
    GYRO.reset();
 
   // autonomousCommand = new ;

    // Setup smart dashboard to choose auton
    m_chooser.setDefaultOption("Auto 0 (Default)", kAuto0);
    m_chooser.addOption("Auto 1", kAuto1);
    m_chooser.addOption("Auto 2", kAuto2);
    m_chooser.addOption("Auto 3", kAuto3);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
   m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_chooser.getSelected());
      // m_autonomousCommand = m_robotContainer.getAutonomousCommand("default");

    // Autos.defaultAuto(new DeviceSubsystem(), new ShooterAnglePIDSubsystem(), new LimelightSubsystem(), new SwerveSubsystem(Constants.m_leftFrontDirection, Constants.m_leftBackDirection,
    // Constants.m_rightFrontDirection, Constants.m_rightBackDirection));

  }
  // Return the instance of the gyroscope
  public static Pigeon2 getGYRO() {
    return GYRO;
  }
  // Zero the gyroscope
  public static void zeroGYRO() {
    GYRO.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_autoSelected);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
