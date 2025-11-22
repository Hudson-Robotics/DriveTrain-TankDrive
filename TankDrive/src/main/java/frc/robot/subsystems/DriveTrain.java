package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private final TalonSRX motor1 = new TalonSRX(1);
    private final TalonSRX motor2 = new TalonSRX(2);
    private final TalonSRX motor3 = new TalonSRX(3);
    private final TalonSRX motor4 = new TalonSRX(4);

    public DriveTrain() {
        // Initialize motors and configurations here
        motor2.setInverted(true);
        motor4.setInverted(true);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        motor1.set(ControlMode.PercentOutput, leftSpeed);
        motor2.set(ControlMode.PercentOutput, leftSpeed);
        motor3.set(ControlMode.PercentOutput, rightSpeed);
        motor4.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
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

    public void cheesyDrive(double forwardSpeed, double rotationSpeed) {
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
