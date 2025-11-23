// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger; // AdvantageKit

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

/**
 * This is a demo program showing the use of tank drive with TalonSRX motors.
 * Supports tank drive, arcade drive, and cheesy drive modes with button switching.
 */
public class Robot extends LoggedRobot {
  // Motor definitions


  // Controller
  private final XboxController drivController = new XboxController(0);

  // Drive mode enumeration
  private enum DriveMode {
    TANK,
    ARCADE,
    CHEESY
  }

  private DriveMode currentMode = DriveMode.TANK;
  private DriveTrain driveTrain;

  private Command autonomousCommand;

  /** Called once at the beginning of the robot program. 
   * @throws ParseException 
   * @throws IOException 
   * @throws FileVersionException */
  public Robot() {
    driveTrain = new DriveTrain();
    autonomousCommand = driveTrain.basicAuto();
  }

  @Override
  public void robotPeriodic() {
    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    if (autonomousCommand != null) {
      driveTrain.resetPose(((PathPlannerAuto) autonomousCommand).getStartingPose());
        autonomousCommand.schedule();
        }
    }
    
    @Override
    public void autonomousPeriodic() {
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

    driveTrain.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Arcade Drive: Left stick Y controls forward/backward, right stick X controls rotation.
   */
  private void arcadeDrive() {
    double forwardSpeed = -drivController.getLeftY();
    double rotationSpeed = drivController.getRightX();

    driveTrain.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  /**
   * Cheesy Drive: Similar to arcade, but with reduced rotation sensitivity at high speeds.
   */
  private void cheesyDrive() {
    double forwardSpeed = -drivController.getLeftY();
    double rotationSpeed = drivController.getRightX();

    driveTrain.cheesyDrive(forwardSpeed, rotationSpeed);
  }

    private void logDriveMode() {
    NetworkTableInstance.getDefault()
        .getTable("Drive")
        .getEntry("Mode")
        .setString(currentMode.toString());

    Logger.recordOutput("Drive/Mode", currentMode.toString());
  }
}
