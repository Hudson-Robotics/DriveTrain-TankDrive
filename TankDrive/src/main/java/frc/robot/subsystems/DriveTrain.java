package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger; 

public class DriveTrain extends SubsystemBase {
    private final TalonSRX motor1 = new TalonSRX(1);
    private final TalonSRX motor2 = new TalonSRX(2);
    private final TalonSRX motor3 = new TalonSRX(3);
    private final TalonSRX motor4 = new TalonSRX(4);

    private double simLeftDistance = 0.0;
    private double simRightDistance = 0.0;
    private double simHeadingRadians = 0.0;

    // --- Simulation constants (replace with your robot’s actual values) ---
    private static final double MAX_RPM = 5330; // CIM motor free speed
    private static final double GEAR_RATIO = 10.71; // drivetrain reduction
    private static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 inches
    private static final double TRACK_WIDTH_METERS = 0.6; // distance between left/right wheels

    private static final double MAX_WHEEL_SPEED = (MAX_RPM / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER_METERS) / 60.0;

          // Odometry for simulated pose
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0, 0);

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

        logDriveSignals(leftSpeed, rightSpeed);
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

        logDriveSignals(leftSpeed, rightSpeed);
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

        logDriveSignals(leftSpeed, rightSpeed);
    }

    private void logDriveSignals(double left, double right) {
        var table = NetworkTableInstance.getDefault().getTable("Drive");

        // Raw outputs and joystick inputs
        table.getEntry("LeftOutput").setDouble(left);
        table.getEntry("RightOutput").setDouble(right);

        Logger.recordOutput("Drive/LeftOutput", left);
        Logger.recordOutput("Drive/RightOutput", right);

        // Convert to wheel speeds (m/s)
        double leftSpeedMps = left * MAX_WHEEL_SPEED;
        double rightSpeedMps = right * MAX_WHEEL_SPEED;

        table.getEntry("LeftSpeedMps").setDouble(leftSpeedMps);
        table.getEntry("RightSpeedMps").setDouble(rightSpeedMps);

        Logger.recordOutput("Drive/LeftSpeedMps", leftSpeedMps);
        Logger.recordOutput("Drive/RightSpeedMps", rightSpeedMps);

        // Linear and angular velocity
        double linearVelocity = (leftSpeedMps + rightSpeedMps) / 2.0;
        double angularVelocity = (rightSpeedMps - leftSpeedMps) / TRACK_WIDTH_METERS;

        table.getEntry("LinearVelocity").setDouble(linearVelocity);
        table.getEntry("AngularVelocity").setDouble(angularVelocity);

        Logger.recordOutput("Drive/LinearVelocity", linearVelocity);
        Logger.recordOutput("Drive/AngularVelocity", angularVelocity);

        // --- Simulation integration ---
        // Each 20ms loop, integrate velocity into cumulative distance
        simLeftDistance += leftSpeedMps * 0.02;
        simRightDistance += rightSpeedMps * 0.02;

        // Integrate angular velocity into heading
        simHeadingRadians += angularVelocity * 0.02;
        Rotation2d simHeading = new Rotation2d(simHeadingRadians);

        // Update odometry with cumulative distances and simulated heading
        odometry.update(simHeading, simLeftDistance, simRightDistance);

        // Log pose (x, y, theta)
        double[] poseArray = {
            odometry.getPoseMeters().getX(),
            odometry.getPoseMeters().getY(),
            odometry.getPoseMeters().getRotation().getDegrees()
        };
        table.getEntry("Pose").setDoubleArray(poseArray);

        Logger.recordOutput("Drive/Pose", odometry.getPoseMeters());
    }
}
