package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToDegrees extends CommandBase {

    private DriveTrain drivetrain;
    private double degrees;
    private int loopsToSettle = 5;
    private int withinThresholdLoops;

    /**
     * Creates a new Rotate command.
     */
    public TurnToDegrees(DriveTrain drivetrain, double degrees) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        withinThresholdLoops = 0;
        drivetrain.turnToDegrees(degrees);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (drivetrain.isAtTurnTarget()) {
            ++withinThresholdLoops;
        } else {
            withinThresholdLoops = 0;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return withinThresholdLoops >= loopsToSettle;
        // return false;
    }
}
