// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPILIB Methods
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Robot Commands
import frc.robot.commands.ShootAngleControlCommand;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.IntakeSequence;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoMovePIDCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.PIDCommandTurnToAngle;
import frc.robot.commands.PIDCompositionDriveCommand;
import frc.robot.commands.PIDGyroCommand;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DeviceSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterAnglePIDSubsystem;
import frc.robot.subsystems.SwerveDirectionPIDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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
  // Front Left Swerve
  private final SwerveDirectionPIDSubsystem m_leftFrontDirection = new SwerveDirectionPIDSubsystem(
    Constants.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN,
    Constants.LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN
  );
  // Back Left Swerve
  private final SwerveDirectionPIDSubsystem m_leftBackDirection = new SwerveDirectionPIDSubsystem(
    Constants.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN,
    Constants.LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN
  );
  // Front Right Swerve
  private final SwerveDirectionPIDSubsystem m_rightFrontDirection = new SwerveDirectionPIDSubsystem(
    Constants.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN,
    Constants.RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN
  );
  // Back Right Swerve
  private final SwerveDirectionPIDSubsystem m_rightBackDirection = new SwerveDirectionPIDSubsystem(
    Constants.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN,
    Constants.RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN
  );
    
  // Initialize Subsystems
  private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem(
    m_leftFrontDirection,
    m_leftBackDirection,
    m_rightFrontDirection, 
    m_rightBackDirection
  );
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final ShooterAnglePIDSubsystem m_AnglePIDSubsystem = new ShooterAnglePIDSubsystem();
  private final DeviceSubsystem m_DeviceSubsystem = new DeviceSubsystem();
  private final LedSubsystem m_LedSubsystem = new LedSubsystem(m_limelight, m_DeviceSubsystem);
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

  // Joystick Controllers
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
  private void configureBindings() {
    // Swerve Command
    SwerveTeleopCommand DriveMode = new SwerveTeleopCommand(
      this.m_swerveDrive, 
      m_driverController::getY, 
      m_driverController::getX,
      m_helperController::getTwist
    );
    this.m_swerveDrive.setDefaultCommand(DriveMode);
    
    // Manually Controlled Shoot Angle Command
    ShootAngleControlCommand AngleControl = new ShootAngleControlCommand(
      this.m_AnglePIDSubsystem, 
      m_limelight,
      m_helperController::getY, 
      m_DeviceSubsystem
    );
    this.m_AnglePIDSubsystem.setDefaultCommand(AngleControl);

    // Toggle Intake Motors to Shoot Note (Shoot Sequence part 2) - BUTTON 2 (HELPER)
    new JoystickButton(m_helperController, 2).onTrue(  
      new InstantCommand(() -> {
        m_DeviceSubsystem.turnIntakeMotors(-1); // -1 is the correct value for shooting
      })
    );    
    new JoystickButton(m_helperController, 2).onFalse(
      new InstantCommand(() -> {
        m_DeviceSubsystem.turnIntakeMotors(0);
      })
    );

    // Toggle Intake Motors - BUTTON 5 (HELPER)
    new JoystickButton(m_helperController, 5).onTrue(
      new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.DEFAULT_SHOOTER_ANGLE)
      );
    new JoystickButton(m_helperController, 5).onFalse(
      new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.DEFAULT_SHOOTER_ANGLE)
    );

    // Toggle Outtake Motors - BUTTON 3 (HELPER)
    new JoystickButton(m_helperController, 3).onTrue(
      new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE)
    );
    new JoystickButton(m_helperController, 3).onFalse(
      new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.DEFAULT_SHOOTER_ANGLE)
    );

    // Shoot into Amp - BUTTON 6 (HELPER)
    new JoystickButton(m_helperController, 6).onTrue(
      new ParallelCommandGroup(
        // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "setAngle", Constants.MIN_SHOOTER_ANGLE),
        // new WaitCommand(1.5),
        // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "ampShoot", Constants.MIN_SHOOTER_ANGLE)
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),
        new PIDGyroCommand(60, m_swerveDrive).withTimeout(.5)
      )
    );

    // Reset Gyro - BUTTON 7 (HELPER)
    new JoystickButton(m_helperController, 7).onTrue(
      new InstantCommand(() -> {
        Robot.zeroGYRO();
      })
    );

    // Align with Limelight - BUTTON 5 (DRIVER)
    new JoystickButton(m_driverController, 5).onTrue(
      new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.5)
    );

    // Toggle Shooter Motors - BUTTON 1 (HELPER)
    new JoystickButton(m_helperController, 1).onTrue(
      new InstantCommand(() -> {
        m_DeviceSubsystem.turnShooterMotors(1);
      })
    );
    new JoystickButton(m_helperController, 1).onFalse(
      new InstantCommand(() -> {
        m_DeviceSubsystem.turnShooterMotors(0);
      })
    );

    // On Hold Manual Arm Aim - BUTTON 10 (HELPER)
    new JoystickButton(m_helperController, 10).onTrue(
      new InstantCommand(() -> {
        m_AnglePIDSubsystem.setManualControl(true);
      })
    );
    new JoystickButton(m_helperController, 10).onFalse(
      new InstantCommand(() -> {
        m_AnglePIDSubsystem.setManualControl(false);
      })
    );

    // Raise pistons - BUTTON 11 (HELPER)
    new JoystickButton(m_helperController, 11).onTrue(
      new InstantCommand(() -> {
        m_ClimbSubsystem.raise();
      })
    );

    // Lower pistons - BUTTON 12 (HELPER)
    new JoystickButton(m_helperController, 12).onTrue(
      new InstantCommand(() -> {
        m_ClimbSubsystem.lower();
      })
    );
  }

  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @ the command to run in autonomous
   */
  public Command getAutonomousCommand(String m_autoSelected, Boolean m_selectedSide) {

    // red is TRUE, bkue is FALSE. i long for the sweet release of death
    if (m_selectedSide) {
      
      if (m_autoSelected.equals("sS")){
        return Autos.RedorBlue_SB_or_SJ(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);

      } else if (m_autoSelected.equals("cC")) {
        return Autos.Red_SI(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
 
      }else if (m_autoSelected.equals("aA")) {
        return Autos.Red_SK(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);

      } else if (m_autoSelected.equals("cCS")) { 
        return Autos.Red_SIJ(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);

      } else if (m_autoSelected.equals("sSA")) { 
        return Autos.Red_SJK(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
        
      } else if (m_autoSelected.equals("sSC")) { 
        return Autos.Red_SJI(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
        
      } else if (m_autoSelected.equals("aAS")) { 
        return Autos.Red_SKJ(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
        
      } else if (m_autoSelected.equals("aAFIVE")) { 
        return Autos.Red_SKH(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
        
      } else if (m_autoSelected.equals("sCSA")) { 
        return Autos.RedorBlue_SABC_or_SKJI(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
        
      } 

    // blue
    } else {
      if (m_autoSelected.equals("sS")){
              return Autos.RedorBlue_SB_or_SJ(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);

      } else if (m_autoSelected.equals("cC")) {
        return Autos.Blue_SA(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);

      } else if (m_autoSelected.equals("aA")) {
        return Autos.Blue_SC(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);

      }else if (m_autoSelected.equals("cCS")) { 
        return Autos.Blue_SAB(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);

      } else if (m_autoSelected.equals("sSA")) {
        return Autos.Blue_SBC(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);

      }else if (m_autoSelected.equals("sSC")) {
        return Autos.Blue_SBA(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);

      }else if (m_autoSelected.equals("aAS")) { 
        return Autos.Blue_SCB(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
        
      } else if (m_autoSelected.equals("aAFIVE")) { 
        return Autos.Blue_SCH(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
        
      } else if (m_autoSelected.equals("cCSA")) { 
        return Autos.RedorBlue_SABC_or_SKJI(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
        
      } 

    
    
    
    }


    if (m_autoSelected.equals("Blue_SA")) {
      return Autos.Blue_SA(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    } 
    else if (m_autoSelected.equals("Blue_SC")) {
      return Autos.Blue_SC(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("Red_SK")) {
      return Autos.Red_SK(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("Red_SI")) {
      return Autos.Red_SI(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("RedorBlue_SB_or_SJ")) {
      return Autos.RedorBlue_SB_or_SJ(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("AutoLeftofDriverGoodTeam")) {
      return Autos.AutoLeftofDriverGoodTeam(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("AutoCenterofDriverGoodTeam")) {
      return Autos.AutoCenterGoodTeam(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("AutoRightofDriverGoodTeam")) {
      return Autos.AutoRightofDriverGoodTeam(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("AutoRightofDriverGoodTeam")) {
      return Autos.AutoRightofDriverGoodTeam(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("Blue_SAD")) {
      return Autos.Blue_SAD(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    //Alliance notes
    else if (m_autoSelected.equals("Blue_SAB")) {
      return Autos.Blue_SAB(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("Blue_SBC")) {
      return Autos.Blue_SBC(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("Blue_SCB")) {
      return Autos.Blue_SCB(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    else if (m_autoSelected.equals("Blue_SBA")) {
      return Autos.Blue_SBA(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    //4 note alliance notes
    else if (m_autoSelected.equals("RedorBlue_SABC_or_SKJI")) {
      return Autos.RedorBlue_SABC_or_SKJI(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }
    

    

    
    
    
    
    
    else {
      return Autos.Blue_SA(m_DeviceSubsystem, m_AnglePIDSubsystem, m_limelight, m_swerveDrive);
    }

  }
}