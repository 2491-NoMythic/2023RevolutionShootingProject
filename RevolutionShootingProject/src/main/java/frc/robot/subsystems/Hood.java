
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Variables;

public class Hood extends SubsystemBase {

    Servo servo1;
    Servo servo2;

    /**
     * Creates a new Shooter.
     */
    public Hood() {

        servo1 = new Servo(Constants.Shooter.servo1PwmID);
        servo2 = new Servo(Constants.Shooter.servo2PwmID);

        SmartDashboard.putNumber("hood position overide", 0);
    }

    /**
     * Set hood position
     * 
     * @param servoUnits (between 0-180 servo units)
     *                   0 is zero degrees and 180 is 270 degrees on servo (maximum
     *                   supported hood position)
     */
    public void setRawHoodPosition(double servoUnits) {
        servo1.setAngle(180 - servoUnits);
        servo2.setAngle(servoUnits);
        SmartDashboard.putNumber("Hood servoUnits", servoUnits);
    }

    public void setHoodPositionDegrees(double degrees) {

        if (degrees > 270) {
            System.out.println("INPUT OVER 270!!!!!!");
            return;
        }

        setRawHoodPosition((degrees * 180 / 270));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("servo1", servo1.getAngle());
        SmartDashboard.putNumber("servo2", servo2.getAngle());

        final double hoodPosition = SmartDashboard.getNumber("hood position overide", 0);

        if (hoodPosition > 0 && hoodPosition != Variables.Shooter.shooterHoodPosition) {
            Variables.Shooter.shooterHoodPosition = hoodPosition;
        }
        SmartDashboard.putNumber("Angle", Variables.Shooter.shooterHoodPosition);
    }
}