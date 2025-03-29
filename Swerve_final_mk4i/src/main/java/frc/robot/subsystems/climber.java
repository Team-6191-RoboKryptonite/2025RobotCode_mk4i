package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class Climber extends SubsystemBase{
  private final SparkMax ClimberMotor = new SparkMax(9, MotorType.kBrushless);
  PIDController pidController = new PIDController(0.2, 0, 0);
  RelativeEncoder ClimberEncoder = ClimberMotor.getEncoder();
  
  public Climber() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    ClimberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ClimberEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ClimberEncoder", ClimberEncoder.getPosition());
  }
  public void setClimberSpeed(double speed) {
    ClimberMotor.set(speed);
  }
  public double getEncoder(){
    return ClimberEncoder.getPosition();
  }
  public double getSpeed(double point, double setpoint){
    double speed = pidController.calculate(point, setpoint);
    return speed;
  }  
}
