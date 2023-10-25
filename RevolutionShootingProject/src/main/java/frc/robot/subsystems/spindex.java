package frc.robot.subsystems;

import static frc.robot.Constants.Spindexer.*;
import static frc.robot.Variables.Spindexer.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class spindex extends SubsystemBase {

	CANSparkMax mainRotationMotor;

	public spindex() {
		mainRotationMotor = new CANSparkMax(mainMotorID, MotorType.kBrushless);
		SmartDashboard.putNumber("Spindexer reverse rotation time", sortModeReverseTime);
		SmartDashboard.putNumber("Spindexer sorting motor power", sortModeMaxPower);
	}

	/**
	 * makes the main part of the spindexer spin
	 * 
	 * @param motorPercent is the speed of the motor
	 * 
	 */
	public void rotate(double motorPercent) {
		mainRotationMotor.set(motorPercent);
		SmartDashboard.putNumber("Spindexer %", motorPercent);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Spindexer Encoder", mainRotationMotor.getEncoder().getVelocity());
		sortModeReverseTime = SmartDashboard.getNumber("Spindexer reverse rotation time", 2);
		sortModeMaxPower = SmartDashboard.getNumber("Spindexer sorting motor power", .5);
	}
}
