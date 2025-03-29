package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake_shooter;

public class Coral_shoot_cmd extends Command {
    private final Intake_shooter Shooter;
    private final double speed;
    private final Timer timer = new Timer();

    public Coral_shoot_cmd(Intake_shooter Shooter, double speed) {
        this.Shooter = Shooter;
        this.speed = speed;
        addRequirements(Shooter);
    }
    @Override
    public void initialize(){
        double pretime = Timer.getFPGATimestamp();
        double nowtime = Timer.getFPGATimestamp();
        while (nowtime - pretime <= 1.5) {
            Shooter.setSpeed(speed);
            nowtime = Timer.getFPGATimestamp();
        }
        end(isFinished()); // need this line to activate ato
    }
    @Override
    public void execute(){
    }
    @Override
    public void end(boolean interrupted) {
        Shooter.setSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return true;
        //return false;
    }
}