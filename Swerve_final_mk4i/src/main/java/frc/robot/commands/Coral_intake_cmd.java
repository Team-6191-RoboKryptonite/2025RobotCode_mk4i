package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral_intake_shoot;
import frc.robot.subsystems.lift;
import frc.robot.subsystems.Coral_intake_shoot;

public class Coral_intake_cmd extends Command {
    private final Coral_intake_shoot Intake;
    private final double speed;


    public Coral_intake_cmd(Coral_intake_shoot Intake, double speed) {
        this.Intake = Intake;
        this.speed = speed;
        addRequirements(Intake);
    }
    @Override
    public void initialize(){}
    @Override
    public void execute(){
        Intake.setIntakeSpeed(speed);
        if(Intake.getPhotosensor() == true){
            Intake.setIntakeSpeed(0);
        }
    }
    @Override
    public void end(boolean interrupted) {
        Intake.setIntakeSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    // final VictorSPX leftshooter = new VictorSPX(29);
    // final VictorSPX rightshooter = new VictorSPX(28);
    // final VictorSPX leftintake = new VictorSPX(24); // 23
    // final VictorSPX rightintake = new VictorSPX(23); // 24

    //     @Override
    //     public void initialize() {
    //         double pretime = Timer.getFPGATimestamp();
    //         double nowtime = Timer.getFPGATimestamp();        
    //         while(nowtime - pretime < 2.0){
    //             rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
    //             leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
    //             rightintake.set(VictorSPXControlMode.PercentOutput, 0);
    //             leftintake.set(VictorSPXControlMode.PercentOutput, 0);
    //             nowtime = Timer.getFPGATimestamp();
    //         }
    //         while (nowtime - pretime > 1.5 && nowtime - pretime < 3.0) {
    //             rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
    //             leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
    //             rightintake.set(VictorSPXControlMode.PercentOutput, -1);
    //             leftintake.set(VictorSPXControlMode.PercentOutput, 1);
    //             nowtime = Timer.getFPGATimestamp();
    //         }
    //         while (nowtime - pretime > 1.0) {
    //             rightshooter.set(VictorSPXControlMode.PercentOutput, 0);
    //             leftshooter.set(VictorSPXControlMode.PercentOutput, 0);
    //             rightintake.set(VictorSPXControlMode.PercentOutput, 0);
    //             leftintake.set(VictorSPXControlMode.PercentOutput, 0);
    //             nowtime = Timer.getFPGATimestamp();
    //             break;
    //         }
    //     }
}
