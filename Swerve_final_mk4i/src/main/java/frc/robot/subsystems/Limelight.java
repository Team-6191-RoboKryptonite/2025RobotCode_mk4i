package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.limelight;

public class Limelight extends SubsystemBase{
  private final NetworkTable table;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry tv;
  //private final NetworkTableEntry ta;
  //private final NetworkTableEntry tid;
  private double DistanceToGoalVertical;
  private double DistanceToGoalHorizontal;
  double VerticalMaxSpeed = limelight.VERTICAL_MAX_SPEED;
  double HorizontalMaxSpeed = limelight.HORIZONTAL_MAX_SPEED;

  public Limelight() {
    this.table = NetworkTableInstance.getDefault().getTable("limelight");
    this.tx = this.table.getEntry("tx");
    this.ty = this.table.getEntry("ty");
    this.tv = this.table.getEntry("tv");
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.putNumber("ty", ty.getDouble(0));
    SmartDashboard.putBoolean("Target Found", tv.getBoolean(false));
  }
  public double getDistanceToGoalVertical() {
    double VerticalOffset = ty.getDouble(0.0);
    // double VerticalOffset = ty.getDouble(0.0) - 2.5;
    double MountAngleDeg = limelight.MOUNT_ANGLE_DEG;
    double LensHeight = limelight.LENS_HEIGHT_METERS;//meter
    double GoalHeight = limelight.GOAL_HEIGHT_METERS;
    double AngleToGoalDeg = MountAngleDeg + VerticalOffset;
    double AngleToGoalRad = AngleToGoalDeg * (Math.PI / 180.0);
    this.DistanceToGoalVertical = Math.abs((GoalHeight - LensHeight) / Math.tan(AngleToGoalRad));
    return this.DistanceToGoalVertical;
  }
  
  public double getDistanceToGoalHorizontal(double distanceToGoalVerticalMeters) {
    double horizontalOffset = this.tx.getDouble(0.0);
    double horizontalOffsetRad = horizontalOffset * (Math.PI / 180.0);
    this.DistanceToGoalHorizontal = (Math.sin(horizontalOffsetRad) * distanceToGoalVerticalMeters) - limelight.HORIZONTAL_OFFSET_METERS;
    return this.DistanceToGoalHorizontal;
  }
  
  public boolean getAprilTag() {
    double tv =  this.tv.getDouble(0.0);
    return tv == 1.0;
  }
  public double getAprilTagtx() {
    double tx = this.tx.getDouble(0);
    return tx;
  }
  public double getAprilTagty() {
    double ty = this.ty.getDouble(0);
    return ty;
  }
  public double getRotationSpeed(double RotationSpeed) {
    double rotationSpeed;
    if(Math.abs(RotationSpeed) > 0.05) {
      rotationSpeed = RotationSpeed;
    }
    else {
      rotationSpeed = 0;
    }
    return rotationSpeed;
  }
  // public double getAprilTagID() {
  //   return this.tid.getDouble(0.0);
  // }
}




