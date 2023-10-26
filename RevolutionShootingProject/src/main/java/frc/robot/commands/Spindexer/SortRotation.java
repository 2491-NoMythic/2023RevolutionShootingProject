package frc.robot.commands.Spindexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Variables;
import frc.robot.subsystems.spindex;

public class SortRotation extends CommandBase {

    private spindex spindexer;

    private Timer timer;

    /**
     * Creates a new Rotate command.
     */
    public SortRotation(spindex spindexer) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.spindexer = spindexer;
        addRequirements(spindexer);

        timer = new Timer();

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double time = timer.get();

        // math.sin is a function that ocelates. you can then pick an x and determain
        // what number should be there
        // here we used a timer to ocelate a motor continuously in a smooth way
        // you can also multply the function to change the speed of the ocelation
        // if you want more information go to https://www.desmos.com/calculator and type
        // in "sin x" then you can multiply it by any nuber you want

        spindexer.rotate((Math.sin(time * Math.PI / Variables.Spindexer.sortModeReverseTime)
                * Math.sin(2 * time * Math.PI / Variables.Spindexer.sortModeReverseTime))
                * Variables.Spindexer.sortModeMaxPower / 2);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        spindexer.rotate(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}