package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.limelight;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;


public class LimelightTrackCmd extends Command{
    private final Swerve swerveSubsystem;
    private final Limelight Limelight;
    private final PIDController flotPid;
    private final PIDController rotationPid;
    private final PIDController horizontalPid;


    public LimelightTrackCmd(Swerve swerveSubsystem, Limelight Limelight) {
        this.swerveSubsystem = swerveSubsystem;
        this.Limelight = Limelight;
        this.flotPid = new PIDController(0.35, 0.08, 0.05);
        this.rotationPid = new PIDController(0.2, 0.09, 0.03);
        this.horizontalPid = new PIDController(0.08, 0, 0.05);
        // this.flotPid = new PIDController(0.35, 0.15, 0);
        // this.rotationPid = new PIDController(0.5, 0.1, 0.15);
        addRequirements(this.swerveSubsystem);
    }
    @Override
    public void execute() {
        boolean tv = Limelight.getAprilTag();
        double tx = Limelight.getAprilTagtx();
        double ta = Limelight.getAprilTagta();
        double ty = Limelight.getAprilTagty();
        double apriltagId = Limelight.getAprilTagID();
        double distanceToGoalVerticalMeters = this.Limelight.getDistanceToGoalVerticalMeters();
        double distanceToGoalHorizontalMeters = MathUtil.applyDeadband(this.Limelight.getDistanceToGoalHorizontalMeters(distanceToGoalVerticalMeters), 0.05);
    
        
        double limelightDistance = 1;
        //double limelightHorizontal = -0.041075960895223;
       
        double verticalSpeed = MathUtil.applyDeadband(
                this.flotPid.calculate(distanceToGoalVerticalMeters), 0.08) * -limelight.VERTICAL_MAX_SPEED;
        // double rotationSpeed = MathUtil.applyDeadband(
        //         this.rotationPid.calculate(distanceToGoalHorizontalMeters), 0.05) * limelight.HORIZONTAL_MAX_SPEED;
        double rotationSpeed = this.rotationPid.calculate(distanceToGoalHorizontalMeters);
        double rotationspeed;
        if(Math.abs(rotationSpeed) > 0.05){
            rotationspeed = rotationSpeed;
        }
        else{
            rotationspeed = 0;
        }
        //double horizontalSpeed = this.horizontalPid.calculate(tx)*-0.15;
        SmartDashboard.putNumber("Calculate Vertical", verticalSpeed);
        SmartDashboard.putNumber("Calculate Rotation", rotationspeed);
        SmartDashboard.putNumber("Caculate VerticalDistance", distanceToGoalVerticalMeters);
        SmartDashboard.putNumber("Caculate HorizontalDistance", distanceToGoalHorizontalMeters);
        //SmartDashboard.putBoolean("Target Found", tv);

        /* Drive */
        
        if (tv) {
            
            if (Math.abs(tx) > 5) {
                 swerveSubsystem.drive(
                    new Translation2d(0,0).times(0), // Constants.Swerve.maxSpeed
                    rotationspeed * 3, // Constants.Swerve.maxAngularVelocity
                    true, 
                    true
                );}
            // } else if (Math.abs(tx) < 15 && Math.abs(tx) >= 5) {
            //     swerveSubsystem.drive(
            //         new Translation2d(0,horizontalSpeed).times(2), // Constants.Swerve.maxSpeed
            //         rotationSpeed * 3, // Constants.Swerve.maxAngularVelocity
            //         true, 
            //         true
            //     );}
            else {
                swerveSubsystem.drive(
                    new Translation2d(verticalSpeed,0).times(3), // Constants.Swerve.maxSpeed
                    rotationspeed * 2, // Constants.Swerve.maxAngularVelocity
                    true, 
                    true
                );
            }
            // swerveSubsystem.drive(
            // new Translation2d(verticalSpeed, 0).times(2.5), // Constants.Swerve.maxSpeed
            // rotationSpeed * 3.5, // Constants.Swerve.maxAngularVelocity
            // true, 
            // true
            // );
            
            // swerveSubsystem.drive(
            //     new Translation2d(verticalSpeed, 0).times(2), // Constants.Swerve.maxSpeed
            //     rotationSpeed * 3, // Constants.Swerve.maxAngularVelocity
            //     true, 
            //     true
            // );
        } else {
            swerveSubsystem.drive(
            new Translation2d(0, 0), //Constants.Swerve.maxSpeed 
            0, 
            true, 
            true
        );// 
        }
    }


    @Override
    public void end(boolean interrupted) {
        // this.swerveSubsystem.stopModules();
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
