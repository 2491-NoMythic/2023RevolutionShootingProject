package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drive extends CommandBase {

	private DriveTrain drivetrain;
	private RobotContainer container;
	double turnSpeed, lastLeftSpeed, lastRightSpeed;
	double currentLeftSpeed = 0;
	double currentRightSpeed = 0;

	/**
	 * Creates a new Drive.
	 */
	public Drive(DriveTrain drivetrain, RobotContainer container) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.drivetrain = drivetrain;
		this.container = container;
		addRequirements(drivetrain);
		SmartDashboard.putNumber("drive train rotate speed", 0.15);
	}

	/**
	 * Prevents acceleration faster than the limit in constants.
	 * 
	 * @return an updated nextSpeed.
	 */
	private double limitAcceleration(double prevSpeed, double nextSpeed) {
		double acceleration = nextSpeed - prevSpeed;
		double signOfAcceleration = Math.signum(acceleration);
		if (Math.abs(acceleration) > Constants.Drivetrain.accelerationSpeed) {
			return prevSpeed + Constants.Drivetrain.accelerationSpeed * signOfAcceleration;
		}
		return nextSpeed;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		CommandJoystick driveController = container.m_driverController;

		turnSpeed = SmartDashboard.getNumber("drive train rotate speed", .15) * Constants.Drivetrain.driveTurnAxis;
		lastLeftSpeed = currentLeftSpeed;
		lastRightSpeed = currentRightSpeed;

		currentLeftSpeed = Constants.Drivetrain.deadzone - turnSpeed;
		currentRightSpeed = Constants.Drivetrain.deadzone + turnSpeed;

		if (SmartDashboard.getBoolean("Record?", false)) {
			System.out.print("{ " + currentLeftSpeed + ", " + currentRightSpeed + " },");
		}

		if (Constants.Drivetrain.useLinerAcceleration) {
			currentLeftSpeed = limitAcceleration(lastLeftSpeed, currentLeftSpeed);
			currentRightSpeed = limitAcceleration(lastRightSpeed, currentRightSpeed);
		}

		drivetrain.drivePercentOutput(currentLeftSpeed, currentRightSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}