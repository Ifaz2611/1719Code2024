// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private static Solenoid m_leftPiston;
    private static Solenoid m_rightPiston;
    // private final Compressor m_Compressor;
  /** Creates a new climbSubsystem. */
  public ClimbSubsystem() {
    // TODO: add constants to constant.java (i did this, but I dont want merge problems). SET THE PISTON IDS AND WELL!!!!!!!!!!
    m_leftPiston= new Solenoid(PneumaticsModuleType.CTREPCM, Constants.m_leftPiston);
    m_rightPiston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.m_rightPiston);
    // m_Compressor = new Compressor(PneumaticsModuleType.CTREPCM)
  }

  // this function should make both the left piston and the right piston raise
  public void raise() {
    m_leftPiston.set(true);
    m_rightPiston.set(true);
  }

  // this function should make them lower
  public void lower(){
    m_leftPiston.set(false);
    m_rightPiston.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
