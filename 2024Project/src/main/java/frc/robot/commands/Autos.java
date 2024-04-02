// In Robot.java (Java) or Robot.h (C++)
package frc.robot.commands;

// WPILIB
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// ROBOT
import frc.robot.subsystems.DeviceSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterAnglePIDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

// PIGEON GYRO
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Autos {
  LimelightSubsystem mLimelightSubsystem;
  ShooterAnglePIDSubsystem m_AnglePIDSubsystem;
  DeviceSubsystem m_DeviceSubsystem;
  SwerveSubsystem m_swerveDrive;

//OLD AUTOS

public static Command Red_SK(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem,
      LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    // SmartDashboard.putNumber("Before Command Sequence", 0);
        //return new ShootSequence(m_DeviceSubsystem);  
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        //ONE NOTE
        new PIDGyroCommand(60, m_swerveDrive).withTimeout(.5),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),

        //TWO NOTE
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.
        MAX_SHOOTER_ANGLE),
        new WaitCommand(1),
        new AutoMovePIDCommand(0, 50, m_swerveDrive).until(m_DeviceSubsystem::checkRing).withTimeout(2),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.
        MAX_SHOOTER_ANGLE).withTimeout(1),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
        new WaitCommand(1),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE),
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),
  
    // //3 NOTE

    // new PIDGyroCommand(0, m_swerveDrive),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0,
    // Constants.MAX_SHOOTER_ANGLE).withTimeout(1),
    // new AutoMovePIDCommand(190, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0, Constants.
    // MAX_SHOOTER_ANGLE),
    // new AutoMovePIDCommand(10, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive).until(m_DeviceSubsystem::checkRing),
    // new PIDCommandTurnToAngle(m_limelight, m_swerveDrive),
    // //new PIDGyroCommand(22, m_swerveDrive),
    // new ShootSequence(m_DeviceSubsystem),
    // new PIDGyroCommand(0, m_swerveDrive),
        new WaitCommand(100) //DO NOT COMMENT
    );
  }

public static Command RedorBlue_SB_or_SJ(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem,
      LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        //ONE NOTE
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),
        //TWO NOTE
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.
        MAX_SHOOTER_ANGLE),
        new WaitCommand(1),
        new AutoMovePIDCommand(0, 40, m_swerveDrive).until(m_DeviceSubsystem::checkRing).withTimeout(2),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.
        MAX_SHOOTER_ANGLE).withTimeout(1),
        new WaitCommand(1),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE),
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),
        

    //3 NOTE

    // new PIDGyroCommand(0, m_swerveDrive),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0,
    // Constants.MAX_SHOOTER_ANGLE),
    // new AutoMovePIDCommand(5, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive).until(m_DeviceSubsystem::checkRing),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0, Constants.
    // MAX_SHOOTER_ANGLE),
    // new AutoMovePIDCommand(185, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive),
    // new PIDCommandTurnToAngle(m_limelight, m_swerveDrive),
    // //new PIDGyroCommand(22, m_swerveDrive),
    // new ShootSequence(m_DeviceSubsystem),
    // new PIDGyroCommand(0, m_swerveDrive),
        new WaitCommand(100) //DO NOT COMMENT
    );
  }

public static Command Red_SI(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem,
      LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
  return new SequentialCommandGroup(
        //AUTO LEFT
        new WaitCommand(.5),
        //ONE NOTE

        //new AutoMovePIDCommand(0,20, m_swerveDrive.returnAverageDistance(), m_swerveDrive).withTimeout(1),
        new PIDGyroCommand(-60, m_swerveDrive).withTimeout(.5),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
        //new PIDGyroCommand(45, m_swerveDrive), keep this commented out
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),

    //TWO NOTE
    
    new PIDGyroCommand(0, m_swerveDrive).withTimeout(1),
    new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn",
    Constants.MAX_SHOOTER_ANGLE),
new WaitCommand(1),
    new AutoMovePIDCommand(0, 40, m_swerveDrive).until(m_DeviceSubsystem::checkRing).withTimeout(2),
    new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.
    MAX_SHOOTER_ANGLE).withTimeout(1),

   // new PIDGyroCommand(30, m_swerveDrive).withTimeout(.8),
    new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
    new WaitCommand(1),
    new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE),
    new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),

    // //3 NOTE

    // new PIDGyroCommand(0, m_swerveDrive),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0,
    // Constants.MAX_SHOOTER_ANGLE),
    // new AutoMovePIDCommand(190, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0, Constants.
    // MAX_SHOOTER_ANGLE),
    // new AutoMovePIDCommand(10, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive),
    // new PIDCommandTurnToAngle(m_limelight, m_swerveDrive),
    // //new PIDGyroCommand(22, m_swerveDrive),
    // new ShootSequence(m_DeviceSubsystem),
    // new PIDGyroCommand(0, m_swerveDrive)
        new WaitCommand(100) //DO NOT COMMENT
    );
    }
    
public static Command Blue_SA(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem,
      LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    // SmartDashboard.putNumber("Before Command Sequence", 0);
        //return new ShootSequence(m_DeviceSubsystem);  
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        //ONE NOTE
        new PIDGyroCommand(60, m_swerveDrive).withTimeout(.5),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),

        //TWO NOTE
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn",
        Constants.MAX_SHOOTER_ANGLE),
        new WaitCommand(1),
        new AutoMovePIDCommand(0, 40, m_swerveDrive).until(m_DeviceSubsystem::checkRing).withTimeout(2),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.
        MAX_SHOOTER_ANGLE).withTimeout(1),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
        new WaitCommand(1),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE),
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),
    // //3 NOTE

    // new PIDGyroCommand(0, m_swerveDrive),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0,
    // Constants.MAX_SHOOTER_ANGLE).withTimeout(1),
    // new AutoMovePIDCommand(190, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0, Constants.
    // MAX_SHOOTER_ANGLE),
    // new AutoMovePIDCommand(10, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive).until(m_DeviceSubsystem::checkRing),
    // new PIDCommandTurnToAngle(m_limelight, m_swerveDrive),
    // //new PIDGyroCommand(22, m_swerveDrive),
    // new ShootSequence(m_DeviceSubsystem),
    // new PIDGyroCommand(0, m_swerveDrive),
         new WaitCommand(100) //DO NOT COMMENT
    );
  }

public static Command Blue_SC(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem,
      LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    // SmartDashboard.putNumber("Before Command Sequence", 0);
        //return new ShootSequence(m_DeviceSubsystem);  
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        //ONE NOTE
        new PIDGyroCommand(-60, m_swerveDrive).withTimeout(.5),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),

        //TWO NOTE
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.
        MAX_SHOOTER_ANGLE),
        new WaitCommand(1),
        new AutoMovePIDCommand(0, 50, m_swerveDrive).until(m_DeviceSubsystem::checkRing).withTimeout(2),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.
        MAX_SHOOTER_ANGLE).withTimeout(1),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
        new WaitCommand(1),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE),
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),
    // //3 NOTE

    // new PIDGyroCommand(0, m_swerveDrive),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0,
    // Constants.MAX_SHOOTER_ANGLE).withTimeout(1),
    // new AutoMovePIDCommand(190, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive),
    // new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, 0, Constants.
    // MAX_SHOOTER_ANGLE),
    // new AutoMovePIDCommand(10, 100, m_swerveDrive.returnAverageDistance(), m_swerveDrive).until(m_DeviceSubsystem::checkRing),
    // new PIDCommandTurnToAngle(m_limelight, m_swerveDrive),
    // //new PIDGyroCommand(22, m_swerveDrive),
    // new ShootSequence(m_DeviceSubsystem),
    // new PIDGyroCommand(0, m_swerveDrive),
         new WaitCommand(100) //DO NOT COMMENT
    );
  }


//NEW AUTOS

public static Command AutoRightofDriverGoodTeam(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem,
      LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
        return new SequentialCommandGroup(
            new WaitCommand(.5),
            //ONE NOTE
            new PIDGyroCommand(60, m_swerveDrive).withTimeout(.5),
            new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
            new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),
            // Allow other robots to do their autonomous sequence
            new WaitCommand(10),
            new AutoMovePIDCommand(0, 40, m_swerveDrive).withTimeout(2),
            new WaitCommand(100) //DO NOT COMMENT
        );
      }

public static Command AutoCenterGoodTeam(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem,
      LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
        return new SequentialCommandGroup(
            new WaitCommand(.5),
            // ONE NOTE
            new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),
            // Allow other robots to do their autonomous sequence
            new WaitCommand(11),
            new AutoMovePIDCommand(0, 40, m_swerveDrive).withTimeout(2),
            new WaitCommand(100) //DO NOT COMMENT
        );
      }

public static Command AutoLeftofDriverGoodTeam(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem,
      LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
        return new SequentialCommandGroup(
            new WaitCommand(.5),
            //ONE NOTE
            new PIDGyroCommand(-60, m_swerveDrive).withTimeout(.5),
            new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1),
            new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),
            // Allow other robots to do their autonomous sequence
            new WaitCommand(10),
            new AutoMovePIDCommand(0, 40, m_swerveDrive).withTimeout(2),
            new WaitCommand(100) //DO NOT COMMENT
        );
      } //CODE FOR AUTOS WITH GOOD TEAMMATES DO NOT UNCODE UNTIL TESTED - neel

// public static Command Test3note(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem,
//       LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
//     return new SequentialCommandGroup(
//         //AUTO LEFT
//         new WaitCommand(.5),
//         //ONE NOTE

//         new ShootSequence(m_DeviceSubsystem).withTimeout(2.4),

//     //TWO NOTE
//     new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.
//         MAX_SHOOTER_ANGLE),
//     new WaitCommand(1.5),
//     new AutoMovePIDCommand(0, 40, m_swerveDrive).until(m_DeviceSubsystem::checkRing).withTimeout(2),
//     new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.
//         MAX_SHOOTER_ANGLE).withTimeout(1),

//    // new PIDGyroCommand(30, m_swerveDrive).withTimeout(.8),
//     new WaitCommand(1),
//     new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE).withTimeout(.5),
//     new ShootSequence(m_DeviceSubsystem).withTimeout(2.6),


//     //Test place
//     new PIDGyroCommand(90, m_swerveDrive).withTimeout(1),
//     new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.
//         MAX_SHOOTER_ANGLE),
//     new WaitCommand(.4),
//     new AutoMovePIDCommand(-87, 40, m_swerveDrive).until(m_DeviceSubsystem::checkRing).withTimeout(1), //Tune
//     new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.
//         MAX_SHOOTER_ANGLE),
//     new PIDGyroCommand(23, m_swerveDrive).withTimeout(.8),
//     new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(.5),
//     new WaitCommand(1),
//     new ShootSequence(m_DeviceSubsystem).withTimeout(2.6),
//     new WaitCommand(100) //DO NOT COMMENT
//     );
//   }

  // Red Amp 3 Note Auto
//   public static Command RedAmp3Note(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
//     return new SequentialCommandGroup(
//         new WaitCommand(.5),
//         // Zero Gyro
//         new InstantCommand(() -> {
//             Robot.zeroGYRO();
//         }),
//         /* FIRST NOTE */
//         new PIDGyroCommand(60, m_swerveDrive).withTimeout(.5), // Turn to 60 degrees
//         new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1), // Align to april tag
//         new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
//         /* SECOND NOTE */
//         new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degree
//         new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
//         new InstantCommand(() -> {
//             m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
//         }),
//         new PIDCompositionDriveCommand(m_swerveDrive, 2, 45, 22).withTimeout(1.5), // Drive to 45 inches
//         new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
//         //new PIDGyroCommand(22, m_swerveDrive).withTimeout(1), // Turn to 22 degrees
//         new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.5), // Align to april tag
//         new WaitCommand(1), // Wait to allow time for arm to adjust
//         new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
//         new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
//         /* THIRD NOTE */
//         new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degrees
//         new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
//         new InstantCommand(() -> {
//             m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
//         }),
//         new PIDCompositionDriveCommand(m_swerveDrive, 4, 190, 0).withTimeout(3), // Drive 190 inches
//         new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
//         new InstantCommand(() -> {
//             m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
//         }),
//         new PIDCompositionDriveCommand(m_swerveDrive, 184, 190, 0).withTimeout(3), // Drive 190 inches with a 200 degree turn
//         new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.75), // Align to april tag
//         new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
//         new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn back to 0
//         new WaitCommand(100) //DO NOT COMMENT
//     );
//   }



//NEW 3 NOTE AUTOS

public static Command Blue_SAD(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new PIDGyroCommand(60, m_swerveDrive).withTimeout(.5), // Turn to 60 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degree
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 2, 40, 22).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new PIDGyroCommand(22, m_swerveDrive).withTimeout(.5), // Turn to 22 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.2), // Align to april tag
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* THIRD NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degrees
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 90, 66, 0).withTimeout(1.5), // Move over 66 inches
        
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 19, 200, 0).withTimeout(3.5), // Drive 200 inches (calculated with 66 inch and 190 inch)
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -161, 200, 0).withTimeout(3.5), // Drive 190 inches with a 200 degree turn
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -90, 66, 0).withTimeout(1.5), // Move over 66 inches
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.75), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn back to 0
        new WaitCommand(100) //DO NOT COMMENT
    );
};

public static Command Blue_SBE(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* THIRD NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 30, 225, 0).withTimeout(1.5), // Move over 66 inches
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(.25),
        new PIDCompositionDriveCommand(m_swerveDrive, -150, 225, 0).withTimeout(3.5), // Drive 200 inches (calculated with 66 inch and 190 inch)
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        new WaitCommand(100) //DO NOT COMMENT
    );
};

public static Command Blue_SCH(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new PIDGyroCommand(-60, m_swerveDrive).withTimeout(.5), // Turn to 60 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degree
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -2, 45, -22).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        //new PIDGyroCommand(22, m_swerveDrive).withTimeout(1), // Turn to -22 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.5), // Align to april tag
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* THIRD NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degrees
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -4, 190, 0).withTimeout(3), // Drive 190 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 176, 190, 0).withTimeout(3), // Drive 190 inches with a 200 degree turn
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.75), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn back to 0
        new WaitCommand(100) //DO NOT COMMENT
    );
  }

public static Command Red_SKH(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new PIDGyroCommand(60, m_swerveDrive).withTimeout(.5), // Turn to 60 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degree
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 2, 45, -22).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        //new PIDGyroCommand(22, m_swerveDrive).withTimeout(1), // Turn to -22 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.5), // Align to april tag
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* THIRD NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degrees
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 4, 190, 0).withTimeout(3), // Drive 190 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -176, 190, 0).withTimeout(3), // Drive 190 inches with a 200 degree turn
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.75), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn back to 0
        new WaitCommand(100) //DO NOT COMMENT
    );
  }

public static Command Red_SJE(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* THIRD NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -30, 225, 0).withTimeout(1.5), // Move over 66 inches
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(.25),
        new PIDCompositionDriveCommand(m_swerveDrive, 150, 225, 0).withTimeout(3.5), // Drive 200 inches (calculated with 66 inch and 190 inch)
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        new WaitCommand(100) //DO NOT COMMENT
    );
};

public static Command Red_SID(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new PIDGyroCommand(-60, m_swerveDrive).withTimeout(.5), // Turn to 60 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degree
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -2, 40, 22).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        //new PIDGyroCommand(22, m_swerveDrive).withTimeout(1), // Turn to 22 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.5), // Align to april tag
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* THIRD NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degrees
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -90, 66, 0).withTimeout(1.5), // Move over 66 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -19, 200, 0).withTimeout(3.5), // Drive 200 inches (calculated with 66 inch and 190 inch)
        
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 161, 200, 0).withTimeout(3.5), // Drive 190 inches with a 200 degree turn
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 90, 66, 0).withTimeout(1.5), // Move over 66 inches
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.75), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn back to 0
        new WaitCommand(100) //DO NOT COMMENT
    );
};


//Alliance Note autos
public static Command Blue_SAB(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new PIDGyroCommand(60, m_swerveDrive).withTimeout(.5), // Turn to 60 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degree
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 2, 40, 22).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new PIDGyroCommand(22, m_swerveDrive).withTimeout(.5), // Turn to 22 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.2), // Align to april tag
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        //THIRD NOTE
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -145, 70, 0).withTimeout(1.5), // Drive to 45 inches
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7) // Shoot
    );
}

public static Command Blue_SBC(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        //THIRD NOTE
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -145, 70, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE),
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new PIDGyroCommand(-22, m_swerveDrive).withTimeout(.5),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(.1),
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7) // Shoot
    );
}

public static Command Blue_SCB(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new PIDGyroCommand(-60, m_swerveDrive).withTimeout(.5), // Turn to 60 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degree
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 2, 40, 22).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new PIDGyroCommand(-22, m_swerveDrive).withTimeout(.5), // Turn to 22 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.2), // Align to april tag
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        //THIRD NOTE
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 145, 70, 0).withTimeout(1.5), // Drive to 45 inches
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7) // Shoot
    );
}

public static Command Blue_SBA(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        //THIRD NOTE
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 145, 70, 0).withTimeout(1.5), // Drive to 45 inches
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new PIDGyroCommand(22, m_swerveDrive).withTimeout(.5),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(.1),
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7) // Shoot
    );
}

/*fix next 4 */
public static Command Red_SKJ(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new PIDGyroCommand(60, m_swerveDrive).withTimeout(.5), // Turn to 60 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degree
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 2, 40, 22).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new PIDGyroCommand(22, m_swerveDrive).withTimeout(.5), // Turn to 22 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.2), // Align to april tag
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        //THIRD NOTE
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -145, 70, 0).withTimeout(1.5), // Drive to 45 inches
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7) // Shoot
    );
}

public static Command Red_SJI(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        //THIRD NOTE
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -145, 70, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE),
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new PIDGyroCommand(-22, m_swerveDrive).withTimeout(.5),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(.1),
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7) // Shoot
    );
}

public static Command Red_SIJ(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new PIDGyroCommand(-60, m_swerveDrive).withTimeout(.5), // Turn to 60 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(1), // Align to april tag
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new PIDGyroCommand(0, m_swerveDrive).withTimeout(1.25), // Turn to 0 degree
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 2, 40, 22).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new PIDGyroCommand(-22, m_swerveDrive).withTimeout(.5), // Turn to 22 degrees
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(0.2), // Align to april tag
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        //THIRD NOTE
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 145, 70, 0).withTimeout(1.5), // Drive to 45 inches
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7) // Shoot
    );
}

public static Command Red_SJK(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        //THIRD NOTE
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 145, 70, 0).withTimeout(1.5), // Drive to 45 inches
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new PIDGyroCommand(22, m_swerveDrive).withTimeout(.5),
        new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(.1),
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7) // Shoot
    );
}

public static Command Blue_SBH(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 40, 0).withTimeout(1.5), // Drive to 45 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(0.25), // Turn intake off
        new WaitCommand(1), // Wait to allow time for arm to adjust
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.DEFAULT_SHOOTER_ANGLE), // Outtake a little
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* THIRD NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), // Turn on intake
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 30, 225, 0).withTimeout(1.5), // Move over 66 inches
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE).withTimeout(.25),
        new PIDCompositionDriveCommand(m_swerveDrive, -150, 225, 0).withTimeout(3.5), // Drive 200 inches (calculated with 66 inch and 190 inch)
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        new WaitCommand(100) //DO NOT COMMENT
    );
};



//4 NOTE
public static Command RedorBlue_SABC_or_SKJI(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
    return new SequentialCommandGroup(
        new WaitCommand(.5),
        // Zero Gyro
        new InstantCommand(() -> {
            Robot.zeroGYRO();
        }),
        /* FIRST NOTE */
        new ShootSequence(m_DeviceSubsystem).withTimeout(1.7), // Shoot
        /* SECOND NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE),
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 35, 65, 22).withTimeout(1.5), // Move over 66 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE),
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", 0),
        new ParallelCommandGroup(
            new PIDCompositionDriveCommand(m_swerveDrive, -145, 65, 0).withTimeout(1.5), // Move over 66 inches
            new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(.25),
            new ShootSequence(m_DeviceSubsystem).withTimeout(2.4)
        ),
        
        /* THIRD NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE),
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 45, 0).withTimeout(1.5), // Move over 66 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE),
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", 0),
        new ParallelCommandGroup(
            new PIDCompositionDriveCommand(m_swerveDrive, 0, 45, 0).withTimeout(1.5), // Move over 66 inches
            new ShootSequence(m_DeviceSubsystem).withTimeout(2.4)
        ),
        /* FOURTH NOTE */
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE),
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new PIDCompositionDriveCommand(m_swerveDrive, -35, 65, -22).withTimeout(1.5), // Move over 66 inches
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOff", Constants.MAX_SHOOTER_ANGLE),
        new InstantCommand(() -> {
            m_swerveDrive.resetDistanceMotors(); // Reset distance motors (CURCIAL TO DRIVING)
        }),
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", 0),
        new ParallelCommandGroup(
            new PIDCompositionDriveCommand(m_swerveDrive, 145, 65, 0).withTimeout(1.5), // Move over 66 inches
            new PIDCommandTurnToAngle(m_limelight, m_swerveDrive).withTimeout(.25),
            new ShootSequence(m_DeviceSubsystem).withTimeout(2.4)
        ),
        
        new WaitCommand(100) //DO NOT COMMENT
    );
};

//Faster Autos

public static Command FastRed_SKH(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
  return new SequentialCommandGroup(    
    new ParallelDeadlineGroup(
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4), 
        new PIDGyroCommand(60, m_swerveDrive).withTimeout(1) //Shoots note 1 and aims to shooter at the same time
    ),
    new InstantCommand(()-> {m_swerveDrive.resetDistanceMotors();}), //resets distance motors (CRUCIAL TO MOVEMENT)
    new ParallelCommandGroup(
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 45, 22).withTimeout(1.5), //Moves, turns on intake and aligns with direction of april tag at the same time
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE),
        new PIDCommandTurnToAngle(m_limelight,m_swerveDrive).withTimeout(1)
    ),

    new InstantCommand(()-> {m_swerveDrive.resetDistanceMotors();}), //resets distance motors (CRUCIAL TO MOVEMENT)
    new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.MAX_SHOOTER_ANGLE).withTimeout(.1), //Outakes
    new ShootSequence(m_DeviceSubsystem).withTimeout(2.4), //Shoots note 2
    new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), //Turns on intake
    new PIDCompositionDriveCommand(m_swerveDrive, 2, 190, 0).withTimeout(3), //Moves back, picks up note
    new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.MAX_SHOOTER_ANGLE).withTimeout(.1), //outtakes note

    new InstantCommand(()-> {m_swerveDrive.resetDistanceMotors();}), //resets distance motors (CRUCIAL TO MOVEMENT)

    new ParallelCommandGroup(
        new PIDCompositionDriveCommand(m_swerveDrive, 182, 190, 0).withTimeout(3), //moves back to origins (very quickly) and shoots note 3 at same time
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4)
    ),
    new WaitCommand(100)
    ); 
};

public static Command FastRed_SKHG(DeviceSubsystem m_DeviceSubsystem, ShooterAnglePIDSubsystem m_AnglePIDSubsystem, LimelightSubsystem m_limelight, SwerveSubsystem m_swerveDrive) {
  return new SequentialCommandGroup(
    
    new ParallelDeadlineGroup(new ShootSequence(m_DeviceSubsystem).withTimeout(2.4), 
        new PIDGyroCommand(60, m_swerveDrive).withTimeout(1) //Shoots note 1 and aims to shooter at the same time
    ),

    new SequentialCommandGroup(new InstantCommand(()-> {m_swerveDrive.resetDistanceMotors();})), //resets distance motors (CRUCIAL TO MOVEMENT)

    new ParallelCommandGroup(
        new PIDCompositionDriveCommand(m_swerveDrive, 0, 45, 22).withTimeout(1.5), //Moves, turns on intake and aligns with direction of april tag at the same time
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE),
        new PIDCommandTurnToAngle(m_limelight,m_swerveDrive).withTimeout(1)

    ),

    new SequentialCommandGroup(new InstantCommand(()-> {m_swerveDrive.resetDistanceMotors();})), //resets distance motors (CRUCIAL TO MOVEMENT)


    new SequentialCommandGroup(
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.MAX_SHOOTER_ANGLE).withTimeout(.1), 
//Outakes
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4), //Shoots note 2
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), //Turns on intake
        new PIDCompositionDriveCommand(m_swerveDrive, 2, 190, 0).withTimeout(3), //Moves back, picks up note
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.MAX_SHOOTER_ANGLE).withTimeout(.1) //outtakes note
    ),

    new SequentialCommandGroup(new InstantCommand(()-> {m_swerveDrive.resetDistanceMotors();})), //resets distance motors (CRUCIAL TO MOVEMENT)

    new ParallelCommandGroup(
        new PIDCompositionDriveCommand(m_swerveDrive, 182, 190, 0).withTimeout(3), //moves back to origins (very quickly) and shoots note 3 at same time
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4)
    ),

    new SequentialCommandGroup(new InstantCommand(()-> {m_swerveDrive.resetDistanceMotors();})), //resets distance motors (CRUCIAL TO MOVEMENT)

    new ParallelCommandGroup(
        new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "intakeOn", Constants.MAX_SHOOTER_ANGLE), //turns on intake and moves back to note at same time
        new PIDCompositionDriveCommand(m_swerveDrive, -12, 190, 0).withTimeout(3.5)
    ),

    new SequentialCommandGroup(new InstantCommand(()-> {m_swerveDrive.resetDistanceMotors();}), new IntakeSequence(m_DeviceSubsystem, m_AnglePIDSubsystem, "outtake", Constants.MAX_SHOOTER_ANGLE)),
    //resets distance motors (CRUCIAL TO MOVEMENT) and slightly outtakes note


    new ParallelCommandGroup(
        new PIDCompositionDriveCommand(m_swerveDrive, 168, 190, 0).withTimeout(3.5), //moves back to origin and shoots note 4 at the same time
        new ShootSequence(m_DeviceSubsystem).withTimeout(2.4)
    )
); 
};













}
