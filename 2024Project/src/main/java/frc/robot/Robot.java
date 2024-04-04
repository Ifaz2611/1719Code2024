// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Phoenix
import com.ctre.phoenix6.hardware.Pigeon2;

// WPILIB
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Autonomous command
  private Command m_autonomousCommand;
  // Robot container
  private RobotContainer m_robotContainer;
  // Gyroscope
  public static Pigeon2 GYRO = new Pigeon2(Constants.CAN_GYRO_PORT);
  // Robot auton choices

  // Chooser for auton modes
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_side = new SendableChooser<>(); // this is for the alliance


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Start camera capture
    CameraServer.startAutomaticCapture();
    UsbCamera USBCAM = CameraServer.startAutomaticCapture();
    USBCAM.setResolution(720, 540);

    // Instantiate our RobotContainer. This will perform all our button bindings
    m_robotContainer = new RobotContainer();
    // Reset gyro
    GYRO.reset();

    // sets up sides
    m_side.setDefaultOption("blue but default", "blue");
    m_side.addOption("Red Side", "red");
    m_side.addOption("Blue Side", "blue");


    // Setup smart dashboard to choose auton
    // m_chooser.setDefaultOption("Center2note", TwoPosition);

    // m_chooser.addOption("RedAmp2note", RedOnePosition);

    // m_chooser.addOption("RedClimber2note", RedThreePosition);
    // m_chooser.addOption("amp 2 note", BlueThreePosition);
    // m_chooser.addOption("Climber 2 note", BlueOnePosition);
    // m_chooser.addOption("Center3note(test)", TEST3NOTE);

    // 2 notes
    m_chooser.addOption("aA", "aA");
    m_chooser.addOption("cC", "cC");
    m_chooser.addOption("sS", "sS");

    m_chooser.addOption("cCS", "cCS");
    m_chooser.addOption("sSA", "sSA");
    m_chooser.addOption("sSC", "sSC");

    m_chooser.addOption("aAS", "aAS");

    m_chooser.addOption("aAFIVE", "aAFIVE");

    // 4 note
    m_chooser.addOption("sCSA","sCSA");

    // auto with good teams i think
    // m_chooser.addOption("AutoRightGoodTeams", AutoLeftGoodTeams);
    // m_chooser.addOption("AutoCenterGoodTeams", AutoCenterGoodTeams);
    // m_chooser.addOption("AutoLeftGoodTeams", AutoRightGoodTeams);


    // m_chooser.addOption("RedAmp3Note", RedAmp3Note);

    // m_chooser.addOption("ThreeNoteRedAmp", ThreeNoteRedAmp);

   


    // Put choices on smart dashboard
    SmartDashboard.putData("Auto choices", m_chooser);
        SmartDashboard.putData("Alliance", m_side);

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
    boolean isRed = m_side.getSelected().equals("red") ? true : false;
    // Get the currently selected auton command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_chooser.getSelected(), isRed);

  }

  // Zero the gyroscope
  public static void zeroGYRO() {
    GYRO.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Schedule the autonomous command 
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
