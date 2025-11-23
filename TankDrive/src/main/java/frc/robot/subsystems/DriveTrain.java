package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class DriveTrain extends SubsystemBase {
    private final TalonSRX motor1 = new TalonSRX(1);
    private final TalonSRX motor2 = new TalonSRX(2);
    private final TalonSRX motor3 = new TalonSRX(3);
    private final TalonSRX motor4 = new TalonSRX(4);

    // Simulation state
    private double simLeftDistance = 0.0;
    private double simRightDistance = 0.0;
    private double simHeadingRadians = 0.0;

    // Instantaneous simulated wheel speeds (m/s)
    private double simLeftSpeedMps = 0.0;
    private double simRightSpeedMps = 0.0;

    // --- Simulation constants (replace with your robot’s actual values) ---
    private static final double MAX_RPM = 5330; // CIM motor free speed
    private static final double GEAR_RATIO = 10.71; // drivetrain reduction
    private static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 inches
    private static final double TRACK_WIDTH_METERS = 0.6; // distance between left/right wheels

    private static final double MAX_WHEEL_SPEED = (MAX_RPM / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER_METERS) / 60.0;

    // Odometry for simulated pose
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(4.0);

    private RobotConfig config;

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    public DriveTrain() {
        motor2.setInverted(true);
        motor4.setInverted(true);

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose2d,              // Robot pose supplier
            this::resetPose,              // Reset odometry to starting pose
            this::getChassisSpeeds,       // Robot-relative chassis speeds (instantaneous)
            (speeds, ff) -> driveRobot(speeds), // Drive callback using ROBOT-RELATIVE ChassisSpeeds
            new PPLTVController(0.02),    // Built-in differential drive controller
            config,
            () -> DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false),
            this
        );
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftSpeed = MathUtil.clamp(shapeInput(leftSpeed), -1, 1);
        rightSpeed = MathUtil.clamp(shapeInput(rightSpeed), -1, 1);

        motor1.set(ControlMode.PercentOutput, leftSpeed);
        motor2.set(ControlMode.PercentOutput, leftSpeed);
        motor3.set(ControlMode.PercentOutput, rightSpeed);
        motor4.set(ControlMode.PercentOutput, rightSpeed);

        logDriveSignals(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
        forwardSpeed = forwardLimiter.calculate(shapeInput(forwardSpeed));
        rotationSpeed = turnLimiter.calculate(shapeInput(rotationSpeed));

        double leftSpeed = MathUtil.clamp(forwardSpeed + rotationSpeed, -1, 1);
        double rightSpeed = MathUtil.clamp(forwardSpeed - rotationSpeed, -1, 1);

        motor1.set(ControlMode.PercentOutput, leftSpeed);
        motor2.set(ControlMode.PercentOutput, leftSpeed);
        motor3.set(ControlMode.PercentOutput, rightSpeed);
        motor4.set(ControlMode.PercentOutput, rightSpeed);

        logDriveSignals(leftSpeed, rightSpeed);
    }

    public void cheesyDrive(double forwardSpeed, double rotationSpeed) {
        forwardSpeed = forwardLimiter.calculate(shapeInput(forwardSpeed));
        rotationSpeed = turnLimiter.calculate(shapeInput(rotationSpeed));

        rotationSpeed = rotationSpeed * (1.0 - (Math.abs(forwardSpeed) * 0.5));

        double leftSpeed = MathUtil.clamp(forwardSpeed + rotationSpeed, -1, 1);
        double rightSpeed = MathUtil.clamp(forwardSpeed - rotationSpeed, -1, 1);

        motor1.set(ControlMode.PercentOutput, leftSpeed);
        motor2.set(ControlMode.PercentOutput, leftSpeed);
        motor3.set(ControlMode.PercentOutput, rightSpeed);
        motor4.set(ControlMode.PercentOutput, rightSpeed);

        logDriveSignals(leftSpeed, rightSpeed);
    }

    @Override
    public void periodic() {
        // Do NOT call logDriveSignals here with motor percents,
        // to avoid double-integrating or clobbering sim speeds unexpectedly.
        // You can still log current pose:
        Logger.recordOutput("Drive/Pose", odometry.getPoseMeters());
    }

    private void logDriveSignals(double leftPercent, double rightPercent) {
        var table = NetworkTableInstance.getDefault().getTable("Drive");

        // Raw outputs
        table.getEntry("LeftOutput").setDouble(leftPercent);
        table.getEntry("RightOutput").setDouble(rightPercent);
        Logger.recordOutput("Drive/LeftOutput", leftPercent);
        Logger.recordOutput("Drive/RightOutput", rightPercent);

        // Convert to instantaneous wheel speeds (m/s) and cache for getChassisSpeeds()
        simLeftSpeedMps = leftPercent * MAX_WHEEL_SPEED;
        simRightSpeedMps = rightPercent * MAX_WHEEL_SPEED;

        table.getEntry("LeftSpeedMps").setDouble(simLeftSpeedMps);
        table.getEntry("RightSpeedMps").setDouble(simRightSpeedMps);
        Logger.recordOutput("Drive/LeftSpeedMps", simLeftSpeedMps);
        Logger.recordOutput("Drive/RightSpeedMps", simRightSpeedMps);

        // Linear and angular velocity
        double linearVelocity = (simLeftSpeedMps + simRightSpeedMps) / 2.0;
        double angularVelocity = (simRightSpeedMps - simLeftSpeedMps) / TRACK_WIDTH_METERS;

        table.getEntry("LinearVelocity").setDouble(linearVelocity);
        table.getEntry("AngularVelocity").setDouble(angularVelocity);
        Logger.recordOutput("Drive/LinearVelocity", linearVelocity);
        Logger.recordOutput("Drive/AngularVelocity", angularVelocity);

        // --- Simulation integration (20ms timestep) ---
        simLeftDistance  += simLeftSpeedMps  * 0.02;
        simRightDistance += simRightSpeedMps * 0.02;
        simHeadingRadians += angularVelocity * 0.02;

        Rotation2d simHeading = new Rotation2d(simHeadingRadians);
        odometry.update(simHeading, simLeftDistance, simRightDistance);

        double[] poseArray = {
            odometry.getPoseMeters().getX(),
            odometry.getPoseMeters().getY(),
            odometry.getPoseMeters().getRotation().getDegrees()
        };
        table.getEntry("Pose").setDoubleArray(poseArray);
        Logger.recordOutput("Drive/Pose", odometry.getPoseMeters());
    }

    private double shapeInput(double input) {
        double deadbanded = MathUtil.applyDeadband(input, 0.05);
        return 0.5 * deadbanded + 0.5 * Math.pow(deadbanded, 3);
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        // Reset sim state to align with the auto start
        simLeftDistance = 0.0;
        simRightDistance = 0.0;
        simHeadingRadians = pose.getRotation().getRadians();
        simLeftSpeedMps = 0.0;
        simRightSpeedMps = 0.0;

        odometry.resetPosition(pose.getRotation(), simLeftDistance, simRightDistance, pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        // Use instantaneous simulated wheel speeds (robot-relative)
        double linearVelocity = (simLeftSpeedMps + simRightSpeedMps) / 2.0;
        double angularVelocity = (simRightSpeedMps - simLeftSpeedMps) / TRACK_WIDTH_METERS;
        return new ChassisSpeeds(linearVelocity, 0.0, angularVelocity);
    }

    public void driveRobot(ChassisSpeeds chassisSpeeds) {
        // Expect ROBOT-RELATIVE speeds; convert to wheel speeds
        var speeds = kinematics.toWheelSpeeds(chassisSpeeds);

        double leftPercent  = MathUtil.clamp(speeds.leftMetersPerSecond  / MAX_WHEEL_SPEED, -1.0, 1.0);
        double rightPercent = MathUtil.clamp(speeds.rightMetersPerSecond / MAX_WHEEL_SPEED, -1.0, 1.0);

        motor1.set(ControlMode.PercentOutput, leftPercent);
        motor2.set(ControlMode.PercentOutput, leftPercent);
        motor3.set(ControlMode.PercentOutput, rightPercent);
        motor4.set(ControlMode.PercentOutput, rightPercent);

        // Critical: update sim via logDriveSignals so autos drive the sim
        logDriveSignals(leftPercent, rightPercent);
    }

    public Command basicAuto() {
        return new PathPlannerAuto("New Auto");
    }
}
