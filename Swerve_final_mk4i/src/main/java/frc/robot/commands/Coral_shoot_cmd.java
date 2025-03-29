package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral_intake_shoot;

public class Coral_shoot_cmd extends Command {
    private final Coral_intake_shoot Shooter;
    private final double speed;


    public Coral_shoot_cmd(Coral_intake_shoot Shooter, double speed) {
        this.Shooter = Shooter;
        this.speed = speed;
        addRequirements(Shooter);
    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        Shooter.setShooterSpeed(speed);
    }
    @Override
    public void end(boolean interrupted) {
        Shooter.setIntakeSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}