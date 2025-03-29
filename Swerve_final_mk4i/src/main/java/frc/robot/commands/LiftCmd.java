// package frc.robot.commands;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.lift;




// public class LiftCmd extends Command {
//     private final lift lift;
//     private final double setpoint;


//     public LiftCmd(lift lift, double setpoint) {
//         this.lift = lift;
//         this.setpoint = setpoint;
//         addRequirements(lift);
//     }


//     @Override
//     public void initialize() {
//         lift.setSpeed(0);
//     }


//     @Override
//     public void execute() {
//         double point = lift.getEncoder();
//         if(Math.abs(point - setpoint) > 0.05){
//             double speed = lift.getSpeed(point, setpoint);
//             lift.setSpeed(speed);}
//         else{
//             lift.setSpeed(0);
//         };
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
