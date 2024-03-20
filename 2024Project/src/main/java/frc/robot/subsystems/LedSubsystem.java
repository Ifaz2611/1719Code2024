// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayDeque;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.proto.System; // WHO THE FUCK WOULD MAKE A CLASS CALLED SYSTEM 
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
// import frc.robot.RobotContainer;

public class LedSubsystem extends SubsystemBase {
  private static Spark m_blinkin = null;
  private LimelightSubsystem m_limelight;
  private double currentValue = Constants.NO_NOTE_BASELINE_GOLD;

  /** Creates a new LedSubsystem. */
  public LedSubsystem(LimelightSubsystem m_lime) {
    m_blinkin = new Spark(Constants.LED_PWM_PIN);
    m_limelight = m_lime;
    // set_led_color(Constants.NO_NOTE_BASELINE_GOLD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double tag = m_limelight.getTag();
    // if (tag == 7 || tag == 4) {
    //   update_led_color(Constants.SPEAKER_GREEN);
    // } else if (tag == 6 || tag == 5) {
    //   update_led_color(Constants.AMP_BLUE);
    // } else {
    //   update_led_color(Constants.NO_NOTE_BASELINE_GOLD);
    // }
    // set_led_color(currentValue);
  }
  // Set all led's to given color
  public void set_led_color(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }
  // public void update_led_color(double val) {
  //   currentValue = val;
  // }

  // Set to alliance color
  // public void allianceColor() {
  //   boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
  //   if (isRed == true){
  //     set_led_color(-0.01); // RED ALLIANCE
  //   } else {
  //     set_led_color(0.19); // BLUE ALLIANCE
  //   }
  // }

  // // checks if the ring is there yessir TODO: GET CORRECT MEASUREMENTS SO TEST ! ! ! ! ! 
  // public boolean checkRing(){
  //   System.out.println("Note sensor get function:" + noteSensor.get());
  //   return noteSensor.get() > 5;
  // }
}
