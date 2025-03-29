package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.autos.*;
import frc.robot.commands.LimelightTrackCmd;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
//import frc.robot.subsystems.lift;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    private final SendableChooser<Command> autoChooser;

    /* Drive Controls */
    private final double translationAxis = driver.getRawAxis(0);
    private final double strafeAxis = driver.getRawAxis(1);
    private final double rotationAxis = driver.getRawAxis(4);
    //private final double setpoint;

    //private final SendableChooser<Command> autoChooser;

    /* Driver Buttons */
    private boolean zerogyro= driver.getRawButton(1);
    //private boolean robotcentric = driver.getRawButton(4);
    //private final JoystickButton robotCentric = new JoystickButton(driver, 4);
    private final JoystickButton intakeButton = new JoystickButton(driver2, 2);
    private final JoystickButton shootButton = new JoystickButton(driver2, 1);
    private final JoystickButton stopButton = new JoystickButton(driver2, 8);
    private final JoystickButton m_button4 = new JoystickButton(driver,4);
    private final JoystickButton mbutton5 = new JoystickButton(driver, 5);
    // private final JoystickButton LiftPlatformButton = new JoystickButton(driver,1);
    // private final JoystickButton LiftlowButton = new JoystickButton(driver,2);
    // private final JoystickButton LiftmiddleButton = new JoystickButton(driver, 3);
    // private final JoystickButton LifthighButton = new JoystickButton(driver, 4)
    // private final JoystickButton m_button1 = new JoystickButton(joystick,1);
    private final Limelight limelight = new Limelight();
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    //private final lift mlift = new lift();
    // private final intake_shoot intake_shooter = new intake_shoot();

    private final LimelightTrackCmd limelightCmd = new LimelightTrackCmd(s_Swerve, limelight);
    //private final LiftCmd liftCmd = new LiftCmd(mlift, setpoint);
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(1), 
                () -> -driver.getRawAxis(0), 
                () -> driver.getRawAxis(4), 
                () -> false
            )
        );
        m_button4.toggleOnTrue(limelightCmd);
        mbutton5.toggleOnTrue(new TeleopSwerve(
            s_Swerve, 
            () -> -driver2.getRawAxis(1)*0.5, 
            () -> -driver2.getRawAxis(0)*0.5, 
            () -> driver2.getRawAxis(4)*0.5, 
            () -> false
        ));
        // NamedCommands.registerCommand("intake_cmd", intake_shooter.intake_cmd());
        // NamedCommands.registerCommand("shooter_cmd", intake_shooter.shooter_cmd());
        // Configure the button bindings
        configureButtonBindings();

        //autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("Auto Mode", autoChooser);

        // SmartDashboard.putData("Field", s_Swerve.field);
        // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {s_Swerve.field.setRobotPose(pose);});
        autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // SmartDashboard.putData("Pathfind to pickup pos", AutoBuilder.pathfindToPose(
        //     new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), new PathConstraints(4, 4, Units.degreesToRadians(360), Units.degreesToRadians(540)),
        //     0, 2.0));
        // SmartDashboard.putData("pathfind to scoring pos", AutoBuilder.pathfindToPose(
        //     new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
        //     new PathConstraints(4, 4, Units.degreesToRadians(360), Units.degreesToRadians(540)), 0, 0));
        // SmartDashboard.putData("on the fly path", Commands.runOnce(() -> {
        //     Pose2d currentPos = s_Swerve.getPose();
        //     Pose2d startPos = new Pose2d(currentPos.getTranslation(), new Rotation2d());
        //     Pose2d endPos = new Pose2d(currentPos.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
            
        //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        //     PathPlannerPath path = new PathPlannerPath(
        //         bezierPoints,
        //         new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
        //         new GoalEndState(0.0, currentPos.getRotation())
        //         );

        //         path.preventFlipping = true;

        //         AutoBuilder.followPath(path).schedule();
        // }));

        /* Driver Buttons */
        //zerogyro.onTrue(new InstantCommand(s_Swerve::zeroHeading));
        
        //LiftPlatformButton.onTrue();
        //shootButton.onTrue(new InstantCommand(() -> intake_shooter.shooter()));
        //stopButton.onTrue(new InstantCommand(() -> intake_shooter.stopevery()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    //PathPlannerPath path = PathPlannerPath.fromPathFile("shooot");
    //return autoChooser.getSelected();
    //return s_Swerve.followPathCommand(path);
// List<PathPlannerPath> pathgroup = PathPlannerAuto.getPathGroupFromAutoFile("auto");
    // Pose2d startingpos = PathPlannerAuto.getStaringPoseFromAutoFile("auto");
    // for (PathPlannerPath path : pathgroup) {
    //     return s_Swerve.followPathCommand(path);
    // }
    }

    // public Command setShootercmd() {
    //     return new intake_shooter_cmd();
    // }

    // public Command setIntakecmd() {
    //     return new intake_cmd();
    // }
}
