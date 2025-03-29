package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.Measure;


public class lift extends SubsystemBase {
    private TalonFX leftMotor1 = new TalonFX(1);
    private TalonFX leftMotor2 = new TalonFX(2);
    private TalonFX rightMotor1 = new TalonFX(4);
    private TalonFX rightMotor2 = new TalonFX(3);
    private DigitalInput topSensor = new DigitalInput(2);  // 上端近接開關 *改端口編號
    private DigitalInput bottomSensor = new DigitalInput(6);  // 下端近接開關 *改端口編號
    private DigitalInput middleSensor1 = new DigitalInput(0); // 中間第一個近接開關 *改端口編號
    private DigitalInput middleSensor2 = new DigitalInput(1); // 中間第二個近接開關 *改端口編號
    public lift(){
    }


    @Override
    public void periodic() {
        // SmartDashboard.putNumber("CANrange distance", point);
        // point = getEncoder();
        SmartDashboard.putBoolean("TopSensor", topSensor.get());
        SmartDashboard.putBoolean("MiddleSensor1", middleSensor1.get());
        SmartDashboard.putBoolean("MiddleSensor2", middleSensor2.get());
        SmartDashboard.putBoolean("BottomSensor", bottomSensor.get());
    }
    private void moveUp(){
        setAllMotors(0.1);
    }
    
    private void moveDown() {
        setAllMotors(-0.1);
    }
    
    private void stopMovement() {
        setAllMotors(0);
    }
    
    private void setAllMotors(double speed) {
        leftMotor1.set(speed);
        leftMotor2.set(speed);
        rightMotor1.set(-speed);
        rightMotor2.set(-speed);
        SmartDashboard.putNumber("speed",speed);
    }

    private boolean getTopSensor(){
        return topSensor.get();
    }
    
    private boolean getMiddle1Sensor(){
        return topSensor.get();
    }
}
