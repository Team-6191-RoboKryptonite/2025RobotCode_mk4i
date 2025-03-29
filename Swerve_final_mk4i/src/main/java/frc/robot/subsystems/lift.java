package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Meters;

import org.ejml.sparse.csc.linsol.qr.LinearSolverQrLeftLooking_DSCC;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.Measure;


public class Lift extends SubsystemBase {
    private TalonFX leftMotor1 = new TalonFX(8);
    private TalonFX rightMotor1 = new TalonFX(9);

    PIDController pid = new PIDController(0.6, 0.03, 0.01);
    public Lift(){
    leftMotor1.setNeutralMode(NeutralModeValue.Brake);
    rightMotor1.setNeutralMode(NeutralModeValue.Brake);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder", getEncoder());
    }
    public double getEncoder(){
        return leftMotor1.getPosition().getValueAsDouble();
    }
    public double getSpeed(double point, double setpoint){
        double speed = pid.calculate(point, setpoint);
        return speed;
    }
    public void setSpeed(double speed) {
        leftMotor1.set(speed * 0.7);
        rightMotor1.set(-speed * 0.7);
        SmartDashboard.putNumber("speed",speed);
    }
    
    // private boolean getMiddle1Sensor(){
    //     return topSensor.get();
    // }
}

