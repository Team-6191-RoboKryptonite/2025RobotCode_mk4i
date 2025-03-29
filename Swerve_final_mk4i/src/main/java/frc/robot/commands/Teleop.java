// package frc.robot.commands;

// import com.ctre.phoenix.motorcontrol.ControlMode;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Lift;
// import frc.robot.subsystems.Intake_shooter;

// public class Teleop extends Command {
//     private final Lift lift;
//     private final Intake_shooter intake_shooter;
//     private final Joystick mdriver2 = new Joystick(1);


//     public Teleop(Lift lift, Intake_shooter intake_shooter){
//         this.lift = lift;
//         this.intake_shooter = intake_shooter;
//         addRequirements(lift, intake_shooter);
//     }
//     @Override
//     public void initialize(){}
//     @Override
//     public void execute(){
// //         //lift up
// //         if(mdriver2.getRawButton(4)){
// //             lift.setSpeed(0.3);
// //         }
// //         //lift down
// //         else if(mdriver2.getRawButton(1)){
// //             lift.setSpeed(-0.3);
// //         }
//         //arm up
//         if(mdriver2.getRawButton(3)){
//            intake_shooter.setArmSpeed(0.3); 
//         }
//         //arm down
//         else if(mdriver2.getRawButton(2)){
//             intake_shooter.setArmSpeed(-0.3); 
//          }
//         //algae shoot
//         else if(mdriver2.getRawButton(1)){
//             intake_shooter.setSpeed(0.3); 
//         }
//         //algae intake
//         else if(mdriver2.getRawButton(4)){
//             intake_shooter.setSpeed(-0.3); 
//         }
// //         //coral shoot
// //         else if(mdriver2.getRawButton(8)){
// //             intake_shooter.setSpeed(-0.3); 
// //         }
//         else{
//             intake_shooter.setArmSpeed(0); 
//             intake_shooter.setSpeed(0); 
//             lift.setSpeed(0);
//         }
//     }
    
//     @Override
//     public void end(boolean interrupted) {
//         lift.setSpeed(0);
//     }
//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }