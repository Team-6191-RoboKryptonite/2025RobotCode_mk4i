package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake_shooter extends SubsystemBase{
    final DigitalInput photoSensor = new DigitalInput(8); // 光電感測器
    final TalonFX CoralIntakeMotor = new TalonFX(12);
    final TalonSRX spinMotor = new TalonSRX(13);
    final Encoder encoder = new Encoder(6, 5);
    final PIDController pid = new PIDController(0.1, 0, 0.01);
    double pretime = Timer.getFPGATimestamp();
    double nowtime = Timer.getFPGATimestamp();

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("photosensor",photoSensor.get());
        SmartDashboard.putNumber("Algae", encoder.getDistance());
    }
    public boolean getPhotosensor(){
        return photoSensor.get();
    }
    public void setSpeed(double speed){
        CoralIntakeMotor.set(speed);
    }
    public void setAlgaeSpeed(double speed){
        spinMotor.set(ControlMode.PercentOutput, speed);
    }
    public double getAlgaeSpeed(double point, double setpoint) {
        double speed = pid.calculate(point, setpoint);
        return speed;
    }
    public double getAlgaeEncoder() {
        return encoder.getDistance() / 5;
    }
}
