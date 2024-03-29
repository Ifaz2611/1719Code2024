/*
 * LedSubsystem: Controls the led lights! Colors are sent in constants!
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPILIB
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

// ROBOT
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {
  private static Spark m_blinkin = null;  
  private LimelightSubsystem m_limelight;

  /** Creates a new LedSubsystem. */
  public LedSubsystem(LimelightSubsystem m_lime) {
    m_blinkin = new Spark(Constants.LED_PWM_PIN);
    m_limelight = m_lime;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: make leds work by putting everything in the periodic method and scheduling it
    // double tag = m_limelight.getTag();
    // set_led_color((tag == 7 || tag == 4) ? Constants.SPEAKER_GREEN : Constants.NO_NOTE_BASELINE_GOLD);
  }

  // Set all led's to given color given a value
  public void set_led_color(double val) {
    if (val >= -1.0 && val <= 1.0) {
      m_blinkin.set(val);
    }
  }
}
