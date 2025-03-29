package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class LiftCmd extends Command {
    private final Lift lift;
    private final double setpoint;

    public LiftCmd(Lift lift, double setpoint) {
        this.lift = lift;
        this.setpoint = setpoint;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        double point = lift.getEncoder();

        while (Math.abs(point - setpoint) > 0.2) {
            point = lift.getEncoder();
            double speed = lift.getSpeed(point, setpoint);
            lift.setSpeed(speed);}
        end(isFinished());
    }

    @Override
    public void execute() {

    }
    
    @Override
    public void end(boolean interrupted) {
        lift.setSpeed(0);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
