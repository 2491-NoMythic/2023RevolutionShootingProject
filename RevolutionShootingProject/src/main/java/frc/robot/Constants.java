// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static final class Drivetrain {
		// TimeDrive Values
		public final static double timeDriveSpeed = .6;
		public final static double timeDriveTime = 2;

		public final static double deadzone = 0.05;

		// drive
		public final static boolean useLinerAcceleration = true;
		public final static double accelerationSpeed = .05;
		public final static int driveTurnAxis = 2;
		public final static int driveMainAxis = 1;
		public final static int driveHorizontalAxis = 0;
		public final static int driveTalonLeftMotor1 = 6;
		public final static int driveTalonLeftMotor2 = 1;
		public final static int driveTalonRightMotor1 = 2;
		public final static int driveTalonRightMotor2 = 3;

		// encoders
		public final static double driveEncoderTicks = 2048.0; // encoder ticks per wheel rotation is 2048
		public final static double driveWheelDiameter = 6.0; // inches
		public final static double driveEncoderToInches = driveWheelDiameter * Math.PI / driveEncoderTicks; // makes
																											// number
																											// inches
		public final static double driveEncoderToMeters = driveEncoderToInches * 39.3701; // 39.3701 is the number of
																							// inches per meter
		public final static double speedModeRPSToTalonOutput = driveEncoderTicks / 10.0;
		public final static double driveEncoderVelocityToRPS = 1.0 / driveEncoderTicks * 10;
		public final static double driveMaxSpeedRPS = 8.0;

		// Distance
		public final class DistanceDrive {
			public final static double kP = 0;
			public final static double kI = 0;
			public final static double kD = 0;
		}

		// odometry
		public final static Pose2d DRIVE_ODOMETRY_ORIGIN = new Pose2d(5.0, 5.0, new Rotation2d());

	}

	public final static int PIGEON_PORT = 2491;

	public static class ShooterSpeeds {
		public static final ShooterSpeeds stop = new ShooterSpeeds(0);
		public static final ShooterSpeeds speed1 = new ShooterSpeeds(6500);
		public static final ShooterSpeeds speed2 = new ShooterSpeeds(7000);
		public static final ShooterSpeeds speed3 = new ShooterSpeeds(7500);
		public static final ShooterSpeeds speed4 = new ShooterSpeeds(8300);

		private double speed;

		public ShooterSpeeds(double speed) {
			this.speed = speed;
		}

		public double getSpeed() {
			return speed;
		}
	}

	public final class Shooter {
		// shooter
		public static final int shooterTalonLeftMotorID = 4;
		public static final int shooterTalonRightMotorID = 5;
		public static final int servo1PwmID = 2;
		public static final int servo2PwmID = 3;
		public final static int photonCannonPwmID = 0;

		// encoders
		public final static double shooterEncoderTicks = 2048.0; // Encoder ticks per wheel rotation is 2048
		public final static double shooterWheelDiameter = 4.0; // Inches
		public final static double shooterEncoderToInches = shooterWheelDiameter * Math.PI / shooterEncoderTicks; // Makes
																													// number
																													// of
																													// inches
		public final static double shooterEncoderVelocityToRPS = 1.0 / shooterEncoderTicks * 10;
		// Speeds
		public static final double shootSpeedRpm = 20000; // Rpm
		public static final double shootSpeedRps = 19500;// TODO change for reality.
		// PID
		public final static int SlotIdx = 0;
		public final static int PIDLoopIdx = 0;
		public final static int TimeoutMs = 0;
		public final static double kP = 2.5;
		public final static double kI = 0.001;
		public final static double kD = 0.4;
		public final static double kF = 0.055;
		public final static int kIzone = 100;
		public final static double PeakOutput = 0;
	}

	// These go from low to high
	public static class ShooterHoodPositions {
		public static final ShooterHoodPositions collapsed = new ShooterHoodPositions(0);
		public static final ShooterHoodPositions position1 = new ShooterHoodPositions(70);
		public static final ShooterHoodPositions position2 = new ShooterHoodPositions(105);
		public static final ShooterHoodPositions position3 = new ShooterHoodPositions(120);
		public static final ShooterHoodPositions position4 = new ShooterHoodPositions(136);

		private double angle;

		public ShooterHoodPositions(double angle) {
			this.angle = angle;
		}

		public double getAngle() {
			return angle;
		}
	}

	public final class Spindexer {
		public static final int mainMotorID = 10;
		public static final int outtakeMotorID = 13;
		public static final int antiJamMotorID = 11;

		public static final double intakeSpindexerSpeed = -.3;
		public static final double shootingSpindexerSpeed = -.09;

		public static final double shootingOutTakeSpeed = 1;
		public static final double antiJamIntakeSpeed = .05;
	}

}