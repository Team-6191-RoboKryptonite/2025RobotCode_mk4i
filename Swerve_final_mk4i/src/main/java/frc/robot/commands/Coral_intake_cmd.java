package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Intake_shooter;

public class Coral_intake_cmd extends Command {
    private final Intake_shooter Intake;
    private final double speed;
    //private final Timer timer = new Timer();

    public Coral_intake_cmd(Intake_shooter Intake, double speed) {
        this.Intake = Intake;
        this.speed = speed;
        addRequirements(Intake);
    }
    @Override
    public void initialize(){
        Intake.setSpeed(0);
    }
    @Override
    public void execute(){
        if(Intake.getPhotosensor() == true){
            Intake.setSpeed(speed);
        } 
        else{
            Intake.setSpeed(0);
        }
        
    }
    @Override
    public void end(boolean interrupted) {
        //Intake.setSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}