package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake_shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Algae_spin_cmd extends Command {
    final Intake_shooter Intake;
    double target_point;

    public Algae_spin_cmd(Intake_shooter Intake, double target_point) {
        this.Intake = Intake;
        this.target_point = target_point;
        addRequirements(Intake);
    }
    @Override
    public void initialize(){
    }
    @Override
    public void execute(){
        double point = Intake.getAlgaeEncoder();
        SmartDashboard.putNumber("Algae2", Intake.getAlgaeEncoder());
        while (Math.abs(point - target_point) > 0.5) {
            point = Intake.getAlgaeEncoder();
            double speed = Intake.getAlgaeSpeed(point, target_point);
            Intake.setAlgaeSpeed(speed);
        }
        end(isFinished());
    }
    @Override
    public void end(boolean interrupted) {
        Intake.setAlgaeSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
