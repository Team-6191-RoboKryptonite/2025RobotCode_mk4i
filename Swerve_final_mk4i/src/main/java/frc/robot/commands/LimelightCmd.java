package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class LimelightCmd extends Command{
    private final Swerve SwerveSubsystem;
    private final Limelight LimelightSubsystem;
    private final PIDController VerticalPid = new PIDController(0.35, 0.08, 0.05);;
    private final PIDController RotationPid = new PIDController(0.2, 0.09, 0.03);;

    public LimelightCmd(Swerve SwerveSubsystem, Limelight LimelightSubsystem) {
        this.SwerveSubsystem = SwerveSubsystem;
        this.LimelightSubsystem = LimelightSubsystem;
        addRequirements(SwerveSubsystem);
    }
    
    @Override
    public void execute() {
        boolean tv = LimelightSubsystem.getAprilTag();
        double tx = LimelightSubsystem.getAprilTagtx();
        //double ty = LimelightSubsystem.getAprilTagty();
        //double apriltagId = Limelight.getAprilTagID();
        double DistanceToGoalVertical = LimelightSubsystem.getDistanceToGoalVertical();//meter
        double DistanceToGoalHorizontal = MathUtil.applyDeadband(LimelightSubsystem.getDistanceToGoalHorizontal(DistanceToGoalVertical), 0.05);//
        double VerticalSpeed = MathUtil.applyDeadband(VerticalPid.calculate(DistanceToGoalVertical), 0.08) * limelight.VERTICAL_MAX_SPEED;
        double RotationSpeed = LimelightSubsystem.getRotationSpeed(RotationPid.calculate(DistanceToGoalHorizontal));
        
        SmartDashboard.putNumber("VerticalSpeed", VerticalSpeed);
        SmartDashboard.putNumber("RotationSpeed", RotationSpeed);
        SmartDashboard.putNumber("DistanceToGoalVertical", DistanceToGoalVertical);
        SmartDashboard.putNumber("DistanceToGoalHorizontal", DistanceToGoalHorizontal);
        
        //Drive
        if(tv) {
            if(Math.abs(tx) > 5) {
                SwerveSubsystem.drive(
                    new Translation2d(0,0), 
                    RotationSpeed * 3, 
                    true, 
                    true
                );
            }
            else {
                SwerveSubsystem.drive(
                    new Translation2d(VerticalSpeed,0).times(3), 
                    RotationSpeed * 2, 
                    true, 
                    true
                );
            }
        } 
        else {
            SwerveSubsystem.drive(
                new Translation2d(0, 0), 
                0, 
                true, 
                true
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
