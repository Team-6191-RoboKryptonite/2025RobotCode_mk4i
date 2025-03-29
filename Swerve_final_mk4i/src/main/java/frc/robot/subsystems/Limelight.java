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
  private final NetworkTableEntry tid;
  private final NetworkTableEntry tv;
  private final NetworkTableEntry ta;
  private double distanceToGoalVerticalMeters;
  private double distanceToGoalHorizontalMeters;
  double VERTICAL_MAX_SPEED = limelight.VERTICAL_MAX_SPEED;
  double HORIZONTAL_MAX_SPEED = limelight.HORIZONTAL_MAX_SPEED;


  public Limelight() {
    this.table = NetworkTableInstance.getDefault().getTable("limelight");
    this.tx = this.table.getEntry("tx");
    this.ty = this.table.getEntry("ty");
    this.tid = this.table.getEntry("tid");
    this.tv = this.table.getEntry("tv");
    this.ta = this.table.getEntry("ta");
  }
  @Override
    public void periodic(){
      SmartDashboard.putNumber("tx", tx.getDouble(0));
      SmartDashboard.putNumber("ty", ty.getDouble(0));
      SmartDashboard.putBoolean("Target Found", tv.getBoolean(false));
    }
    public double getDistanceToGoalVerticalMeters() {
      double verticalOffset = this.ty.getDouble(0.0)-2.5;

      double mountAngleDeg = limelight.MOUNT_ANGLE_DEG;
      double lensHeightMeters = limelight.LENS_HEIGHT_METERS;
      double goalHeightMeters = limelight.GOAL_HEIGHT_METERS;
 
      double angleToGoalDeg = mountAngleDeg + verticalOffset;
      double angleToGoalRad = angleToGoalDeg * (Math.PI / 180.0);
      this.distanceToGoalVerticalMeters = Math.abs((goalHeightMeters - lensHeightMeters) / Math.tan(angleToGoalRad));
 
      return this.distanceToGoalVerticalMeters;
  }
  public double getDistanceToGoalHorizontalMeters(double distanceToGoalVerticalMeters) {
      // if (distanceToGoalVerticalMeters == -1) {
      //     distanceToGoalVerticalMeters = this.getDistanceToGoalVerticalMeters();
      // }
      double horizontalOffset = this.tx.getDouble(0.0);
 
      double horizontalOffsetRad = horizontalOffset * (Math.PI / 180.0);
 
      this.distanceToGoalHorizontalMeters = (Math.sin(horizontalOffsetRad) * distanceToGoalVerticalMeters) - limelight.HORIZONTAL_OFFSET_METERS;
      return this.distanceToGoalHorizontalMeters;
  }
  public double getAprilTagID() {
    return this.tid.getDouble(0.0);
  }
  public boolean getAprilTag() {
    double tv =  this.tv.getDouble(0.0);
    return tv == 1.0;
  }
  public double getAprilTagta() {
    double ta = this.ta.getDouble(0);
    return ta;
  }
  public double getAprilTagtx() {
    double tx = this.tx.getDouble(0);
    return tx;
  }
  public double getAprilTagty() {
    double ty = this.ty.getDouble(0);
    return ty;
  }
}




