// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a demo program showing the use of tank drive with TalonSRX motors.
 * Supports tank drive, arcade drive, and cheesy drive modes with button switching.
 */
public class Robot extends TimedRobot {
  // Motor definitions
  private final TalonSRX motor1 = new TalonSRX(1);
  private final TalonSRX motor2 = new TalonSRX(2);
  private final TalonSRX motor3 = new TalonSRX(3);
  private final TalonSRX motor4 = new TalonSRX(4);

  // Controller
  private final XboxController drivController = new XboxController(0);

  // Drive mode enumeration
  private enum DriveMode {
    TANK,
    ARCADE,
    CHEESY
  }

  private DriveMode currentMode = DriveMode.TANK;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // Invert right side motors so that positive voltages result in both sides moving forward
    motor2.setInverted(true);
    motor4.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Check for drive mode switching buttons
    if (drivController.getAButton()) {
      currentMode = DriveMode.TANK;
    } else if (drivController.getBButton()) {
      currentMode = DriveMode.ARCADE;
    } else if (drivController.getXButton()) {
      currentMode = DriveMode.CHEESY;
    }

    // Execute the current drive mode
    switch (currentMode) {
      case TANK:
        tankDrive();
        break;
      case ARCADE:
        arcadeDrive();
        break;
      case CHEESY:
        cheesyDrive();
        break;
    }
  }

  /**
   * Tank Drive: Left joystick controls left side, right joystick controls right side.
   */
  private void tankDrive() {
    double leftSpeed = -drivController.getLeftY();
    double rightSpeed = -drivController.getRightY();

    motor1.set(ControlMode.PercentOutput, leftSpeed);
    motor2.set(ControlMode.PercentOutput, leftSpeed);
    motor3.set(ControlMode.PercentOutput, rightSpeed);
    motor4.set(ControlMode.PercentOutput, rightSpeed);
  }

  /**
   * Arcade Drive: Left stick Y controls forward/backward, right stick X controls rotation.
   */
  private void arcadeDrive() {
    double forwardSpeed = -drivController.getLeftY();
    double rotationSpeed = drivController.getRightX();

    // Mix forward and rotation for arcade drive
    double leftSpeed = forwardSpeed + rotationSpeed;
    double rightSpeed = forwardSpeed - rotationSpeed;

    // Clamp values between -1 and 1
    leftSpeed = Math.max(-1, Math.min(1, leftSpeed));
    rightSpeed = Math.max(-1, Math.min(1, rightSpeed));

    motor1.set(ControlMode.PercentOutput, leftSpeed);
    motor2.set(ControlMode.PercentOutput, leftSpeed);
    motor3.set(ControlMode.PercentOutput, rightSpeed);
    motor4.set(ControlMode.PercentOutput, rightSpeed);
  }

  /**
   * Cheesy Drive: Similar to arcade, but with reduced rotation sensitivity at high speeds.
   */
  private void cheesyDrive() {
    double forwardSpeed = -drivController.getLeftY();
    double rotationSpeed = drivController.getRightX();

    // Apply reduced sensitivity to rotation (cheesy drive characteristic)
    rotationSpeed = rotationSpeed * (1.0 - (Math.abs(forwardSpeed) * 0.5));

    double leftSpeed = forwardSpeed + rotationSpeed;
    double rightSpeed = forwardSpeed - rotationSpeed;

    // Clamp values between -1 and 1
    leftSpeed = Math.max(-1, Math.min(1, leftSpeed));
    rightSpeed = Math.max(-1, Math.min(1, rightSpeed));

    motor1.set(ControlMode.PercentOutput, leftSpeed);
    motor2.set(ControlMode.PercentOutput, leftSpeed);
    motor3.set(ControlMode.PercentOutput, rightSpeed);
    motor4.set(ControlMode.PercentOutput, rightSpeed);
  }
}
