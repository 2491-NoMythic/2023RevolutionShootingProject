package frc.robot.subsystems;

import static frc.robot.Constants.Spindexer.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {

	CANSparkMax outtakeMotor;

	public Outtake() {
		outtakeMotor = new CANSparkMax(outtakeMotorID, MotorType.kBrushless);
	}

	/**
	 * runs outtake motor
	 * @param motorPercent is the speed of the motor
	 */
	public void runOuttakeMotor(double motorPercent) {
		outtakeMotor.set(motorPercent);
		SmartDashboard.putNumber("Outtake %", motorPercent);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Outtake Encoder", outtakeMotor.getEncoder().getVelocity());
	}
	
}