package frc.robot.commands.Spindexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.spindex;

public class ShootingRotation extends CommandBase {

    private spindex spindexer;

    /**
     * Creates a new Shooting Rotate command.
     * Spins the spindexer at Constants.Spindexer.shootingSpindexerSpeed
     */
    public ShootingRotation(spindex spindexer) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.spindexer = spindexer;
        addRequirements(spindexer);

        SmartDashboard.putNumber("spindexer Shooting speed", -.21);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double shootSpeed = SmartDashboard.getNumber("spindexer Shooting speed", -.21);
        spindexer.rotate(shootSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        spindexer.rotate(Constants.ShooterSpeeds.stop.getSpeed());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
