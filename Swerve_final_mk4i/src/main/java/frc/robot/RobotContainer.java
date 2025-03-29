package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DriverStation;
//import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    //private final Joystick driver2 = new Joystick(1);

    private final SendableChooser<Command> autoChooser;

    /* Drive Controls */
    private final double translationAxis = driver.getRawAxis(0);
    private final double strafeAxis = driver.getRawAxis(1);
    private final double rotationAxis = driver.getRawAxis(4);


    /* Driver Buttons */
    // private final JoystickButton bottom = new JoystickButton(driver, 1);
    private final JoystickButton middle1 = new JoystickButton(driver, 6);
    // private final JoystickButton middle2 = new JoystickButton(driver, 3);
    private final JoystickButton top = new JoystickButton(driver, 2);
    // private final JoystickButton shooter = new JoystickButton(driver, 2);
    // private final JoystickButton intake = new JoystickButton(driver, 3);
    //private final JoystickButton climberup = new JoystickButton(driver, 5);
    //private final JoystickButton climberdown = new JoystickButton(driver, 6);
    // private final JoystickButton algae_intake1 = new JoystickButton(driver, 5);
    // private final JoystickButton algae_intake2 = new JoystickButton(driver2, 6);
    // private final JoystickButton algae_intake3 = new JoystickButton(driver2, 5);
    // private final JoystickButton m_button5 = new JoystickButton(driver,5);
    // private final JoystickButton m_button8 = new JoystickButton(driver,8);
    private final JoystickButton coral_intake = new JoystickButton(driver, 1);
    private final JoystickButton coral_shooter = new JoystickButton(driver, 3);
    // private final JoystickButton algae_shooter1 = new JoystickButton(driver2, 4);
    // private final JoystickButton algae_shooter2 = new JoystickButton(driver2, 1);
    private final JoystickButton limelightButton = new JoystickButton(driver,4);
    private final Limelight limelight = new Limelight();
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Lift mlift = new Lift();
    private final Intake_shooter intake_shooter = new Intake_shooter();
    private final Climber climber = new Climber();
    double speed;
    /* Commands */
    private final LimelightCmd limelightCmd = new LimelightCmd(s_Swerve, limelight);
    // private final AutoCmd autoCmd = new AutoCmd(mlift, intake_shooter, 1.1);
    // private final Coral_intake_cmd coral_intake_cmd = new Coral_intake_cmd(intake_shooter, 0.3);
    // private final Teleop teleop = new Teleop(mlift, intake_shooter);
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //mlift.setDefaultCommand(new Teleop(mlift, intake_shooter));
        //intake_shooter.setDefaultCommand(new Teleop(mlift, intake_shooter));

        intake_shooter.setDefaultCommand(new Algae_spin_cmd(intake_shooter, 6));
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(0), 
                () -> -driver.getRawAxis(1), 
                () -> -driver.getRawAxis(4), 
                () -> false
            )
        );
        //mlift.setDefaultCommand(new LiftTeleop(mlift));
        // m_button5.onTrue(new Coral_intake_cmd(intake_shooter, 0.2));
        // m_button8.toggleOnTrue(new TeleopSwerve(
        //     s_Swerve, 
        //     () -> driver2.getRawAxis(0)*0.3, 
        //     () -> -driver2.getRawAxis(1)*0.3, 
        //     () -> -driver2.getRawAxis(4)*0.3,
        //     () -> true
        // ));
        // m_button6.onTrue(new Coral_shoot_cmd(intake_shooter, 0.2));
        limelightButton.toggleOnTrue(limelightCmd);

        // Configure the button bindings
        configureButtonBindings();
        // NamedCommands.registerCommand("Auto1", autoCmd);
        // NamedCommands.registerCommand("intake", coral_intake_cmd);
        NamedCommands.registerCommand("shoot", new Coral_shoot_cmd(intake_shooter, 0.3));
        autoChooser = AutoBuilder.buildAutoChooser();
        // NamedCommands.registerCommand("stop", new Coral_shoot_cmd(intake_shooter, 0));
        // NamedCommands.registerCommand("shooter", new LiftCmd(mlift, 0).andThen(new Coral_shoot_cmd(intake_shooter, 0.3)).andThen(new LiftCmd(mlift, 0)));
        // NamedCommands.registerCommand("down", new LiftCmd(mlift, 0));
        // NamedCommands.registerCommand("intake", new Coral_intake_cmd(intake_shooter, 0.3));
        SmartDashboard.putBoolean("photosensor", intake_shooter.getPhotosensor());
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        top.onTrue(new LiftCmd(mlift, 1.1).andThen(new Coral_shoot_cmd(intake_shooter, -0.2)).andThen(new LiftCmd(mlift, 0.08)));
        middle1.onTrue(new LiftCmd(mlift, 3).andThen(new Coral_shoot_cmd(intake_shooter, -0.2)).andThen(new LiftCmd(mlift, 0.1)));
        //middle2.onTrue(new LiftCmd(mlift, 1.1).andThen(new Coral_shoot_cmd(intake_shooter, -0.2)).andThen(new LiftCmd(mlift, 0.3)));
        // bottom.onTrue(new LiftCmd(mlift, 0.1).andThen(new Coral_shoot_cmd(intake_shooter, -0.2)));
        // shooter.onTrue(new Coral_shoot_cmd(intake_shooter, -0.2));
        coral_intake.whileTrue(new Coral_intake_cmd(intake_shooter,-0.1));
        coral_intake.whileFalse(new Coral_intake_cmd(intake_shooter,0));
        coral_shooter.onTrue(new Coral_shoot_cmd(intake_shooter, -0.1));
        // climberup.whileTrue(new ClimberSetSPeed(climber, 0.3));
        // climberdown.whileTrue(new ClimberSetSPeed(climber, -0.3));
        // climberup.whileFalse(new ClimberSetSPeed(climber, 0));
        // climberdown.whileFalse(new ClimberSetSPeed(climber, 0));
        // algae_intake1.onTrue(new LiftCmd(mlift, 2.59).andThen(new Algae_spin_cmd(intake_shooter, 60)).andThen(new Coral_shoot_cmd(intake_shooter, -0.3)).andThen(new LiftCmd(mlift, 0)));
        // algae_intake2.onTrue(new LiftCmd(mlift, 1.94).andThen(new Algae_spin_cmd(intake_shooter, 60)).andThen(new Coral_shoot_cmd(intake_shooter, -0.3)).andThen(new LiftCmd(mlift, 0)));
        // algae_intake3.onTrue(new Algae_spin_cmd(intake_shooter, 72).andThen(new Coral_shoot_cmd(intake_shooter, 0.3)).andThen(new Algae_spin_cmd(intake_shooter, 8)));
        // intake.whileTrue(new Coral_intake_cmd(intake_shooter, -0.1));
        // intake.whileFalse(new Coral_intake_cmd(intake_shooter, 0));
        // algae_shooter1.onTrue(new Coral_shoot_cmd(intake_shooter, 0.5));
        // algae_shooter2.onTrue(new Coral_shoot_cmd(intake_shooter, -0.5));
        // intake.whileTrue(new Coral_intake_cmd(intake_shooter));
        // shooter.whileTrue(limelightCmd);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

