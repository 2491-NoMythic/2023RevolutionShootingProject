package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.hardware.TalonFX;
import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.*;
import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CustomDifferentialDriveOdometry;
import frc.robot.LimelightValues;
import frc.robot.RobotContainer;
import frc.robot.Variables;

public class DriveTrain extends SubsystemBase {
	private WPI_TalonFX driveLeftMotor1, driveLeftMotor2, driveRightMotor1, driveRightMotor2;
	private Pigeon2 gyro;
	private CustomDifferentialDriveOdometry odometry;
	Limelight limelight = Limelight.getInstance();
	private final Field2d m_field = new Field2d();

	private Boolean forceTrustLimelight = false;
	private Boolean useLimelight = true;

	public DriveTrain() {

		SmartDashboard.putData("Field", m_field);
		SmartDashboard.putData("resetOdometry", new InstantCommand(() -> this.resetOdometry()));
		// creating motors
		driveLeftMotor1 = new WPI_TalonFX(driveTalonLeftMotor1);
		driveLeftMotor2 = new WPI_TalonFX(driveTalonLeftMotor2);
		driveRightMotor1 = new WPI_TalonFX(driveTalonRightMotor1);
		driveRightMotor2 = new WPI_TalonFX(driveTalonRightMotor2);

		driveRightMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		driveLeftMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

		// Setting Followers
		driveLeftMotor2.follow(driveLeftMotor1);
		driveRightMotor2.follow(driveRightMotor1);

		// making right motors go right
		driveRightMotor1.setInverted(true);
		driveRightMotor2.setInverted(true);
		driveLeftMotor1.setInverted(false);
		driveLeftMotor2.setInverted(false);

		gyro = new Pigeon2(PIGEON_PORT);

		resetGyro();

		resetEncoders();
		odometry = new CustomDifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle()), driveEncoderToMeters,
				driveEncoderToMeters);

		SmartDashboard.putNumber("Rotate P", Variables.Drivetrain.RotationCommand.kP);
		SmartDashboard.putNumber("Rotate I", Variables.Drivetrain.RotationCommand.kI);
		SmartDashboard.putNumber("Rotate D", Variables.Drivetrain.RotationCommand.kD);
	}

	// gyro tingz
	public void resetGyro() {
		gyro.setYaw(0);
	}

	public Rotation2d getGyroscopeRotation() {
		return gyro.getRotation2d();
	}

	public double getGyroAngle() {
		return (getRawGyroAngle() % 360 + 360) % 360;
	}

	public double getRawGyroAngle() {
		return gyro.getAngle();
	}

	public WPI_TalonFX[] getTalonFX() {
		return new WPI_TalonFX[] { driveLeftMotor1, driveLeftMotor2, driveRightMotor1, driveRightMotor2 };
	}

	// pose tingz
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	// Returns wheel speeds of the robot
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
	}

	// Resets odometry to the specified pose
	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		odometry.resetPosition(gyro.getRotation2d(), getLeftEncoderDistanceMeeters(), getRightEncoderDistanceMeeters(),
				pose);
	}

	// creating percent output for both right and left
	public void drivePercentOutput(double speed) {
		drivePercentOutput(speed, speed);
	}

	public void drivePercentOutput(double leftSpeed, double rightSpeed) {
		driveLeftPercentOutput(leftSpeed);
		driveRightPercentOutput(rightSpeed);
	}

	public void driveLeftPercentOutput(double speed) {
		driveLeftMotor1.set(ControlMode.PercentOutput, speed);
	}

	public void driveRightPercentOutput(double speed) {
		driveRightMotor1.set(ControlMode.PercentOutput, speed);
	}

	// createing voltge output for both right and left.
	public void driveVoltageOutput(double speed) {
		driveVoltageOutput(speed, speed);
	}

	public void driveVoltageOutput(double leftSpeed, double rightSpeed) {
		driveLeftVoltageOutput(leftSpeed);
		driveRightVoltageOutput(rightSpeed);
	}

	public void driveLeftVoltageOutput(double speed) {
		driveLeftMotor1.setVoltage(speed);
	}

	public void driveRightVoltageOutput(double speed) {
		driveRightMotor1.setVoltage(speed);
	}

	// creating drive velocity for both right and left
	public void driveVelocity(double speed) {
		driveVelocity(speed, speed);
	}

	public void driveVelocity(double leftSpeed, double rightSpeed) {
		driveLeftVelocity(leftSpeed);
		driveRightVelocity(rightSpeed);
	}

	public void driveLeftVelocity(double speed) {
		driveLeftMotor1.set(ControlMode.Velocity, speed);
	}

	public void driveRightVelocity(double speed) {
		driveRightMotor1.set(ControlMode.Velocity, speed);
	}

	public double getRightDriveSpeed() {
		return driveRightMotor1.get();
	}

	public double getLeftDriveSpeed() {
		return driveLeftMotor1.get();
	}

	// robot can stop
	public void stop() {
		drivePercentOutput(0, 0);
	}

	public void breakModeOn(Boolean b) {
		if (b) {
			driveLeftMotor1.setNeutralMode(NeutralMode.Brake);
			driveRightMotor1.setNeutralMode(NeutralMode.Brake);
		} else {
			driveLeftMotor1.setNeutralMode(NeutralMode.Coast);
			driveRightMotor1.setNeutralMode(NeutralMode.Coast);
		}
	}

	// getting encoder distance and rate
	public double getRightEncoderDistanceInches() {
		return driveRightMotor1.getSelectedSensorPosition(0) * driveEncoderToInches;
	}

	public double getLeftEncoderDistanceInches() {
		return driveLeftMotor1.getSelectedSensorPosition(0) * driveEncoderToInches;
	}

	public double getLeftEncoderDistanceMeeters() {
		return getLeftEncoderDistanceInches() * 39.37;
	}

	public double getRightEncoderDistanceMeeters() {
		return getRightEncoderDistanceInches() * 39.7;
	}

	public double getLeftEncoderDistanceRaw() {
		return driveLeftMotor1.getSelectedSensorPosition(0);
	}

	public double getRightEncoderDistanceRaw() {
		return driveRightMotor1.getSelectedSensorPosition(0);
	}

	public double getEncoderDistance() {
		return ((getLeftEncoderDistanceInches() + getRightEncoderDistanceInches()) / 2);
	}

	public double getLeftEncoderRate() {
		return driveLeftMotor1.getSelectedSensorVelocity(0) * driveEncoderVelocityToRPS;
	}

	public double getRightEncoderRate() {
		return driveRightMotor1.getSelectedSensorVelocity(0) * driveEncoderVelocityToRPS;
	}

	public double getEncoderRate() {
		return ((getRightEncoderRate() + getLeftEncoderRate()) / 2);
	}

	public void resetEncoders() {
		driveRightMotor1.setSelectedSensorPosition(0);
		driveLeftMotor1.setSelectedSensorPosition(0);
	}

	public void updateOdometry() {
		// odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation());
		odometry.update(getGyroscopeRotation(), getLeftEncoderDistanceMeeters(), getRightEncoderDistanceMeeters());
	}

	public void updateOdometryWithVision(Pose2d estematedPose, double timestampSeconds) {
		odometry.directUpdate(estematedPose);
		// odometry.updateOdometryWithVision();
	}

	public void resetOdometry() {
		odometry.resetPosition(getGyroscopeRotation(), getLeftEncoderDistanceMeeters(),
				getRightEncoderDistanceMeeters(), DRIVE_ODOMETRY_ORIGIN);
	}

	public void resetOdometryFromVision(Pose2d pose) {
		odometry.resetPosition(getGyroscopeRotation(), getLeftEncoderDistanceMeeters(),
				getRightEncoderDistanceMeeters(), pose);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Angle", getGyroAngle());
		if ((SmartDashboard.getNumber("Rotate P", 0) != Variables.Drivetrain.RotationCommand.kP)) {
			Variables.Drivetrain.RotationCommand.kP = SmartDashboard.getNumber("Rotate P", 0);
		}
		if ((SmartDashboard.getNumber("Rotate I", 0) != Variables.Drivetrain.RotationCommand.kI)) {
			Variables.Drivetrain.RotationCommand.kI = SmartDashboard.getNumber("Rotate I", 0);
		}
		if ((SmartDashboard.getNumber("Rotate D", 0) != Variables.Drivetrain.RotationCommand.kD)) {
			Variables.Drivetrain.RotationCommand.kD = SmartDashboard.getNumber("Rotate D", 0);
		}
		// This method will be called once per scheduler run

		// Update the odometry in the periodic block
		odometry.update(gyro.getRotation2d(), getLeftEncoderDistanceInches(), getRightEncoderDistanceInches()); // TODO
																												// Test
																												// encoder
		// distance method,
		// check gearing
		// ratio between
		// distanceand
		// encoder

		updateOdometry();
		if (useLimelight) {
			LimelightValues visionData = limelight.getLimelightValues();
			Boolean isVisionValid = visionData.isResultValid;
			Boolean isVisionTrustworthy = isVisionValid
					&& visionData.isPoseTrustworthy(odometry.getPoseMeters());
			SmartDashboard.putBoolean("visionValid", isVisionTrustworthy);
			if (isVisionTrustworthy || (forceTrustLimelight && isVisionValid)) {
				updateOdometryWithVision(visionData.getbotPose(), visionData.gettimestamp());
			}
		}
		m_field.setRobotPose(odometry.getPoseMeters());
		SmartDashboard.putNumber("Robot Angle", getGyroscopeRotation().getDegrees());
		SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

	}
}
