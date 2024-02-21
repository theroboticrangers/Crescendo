// Copyright (c) FI// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LauncherConstants;
//import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.Constants.RobotStates;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.commands.Grab;
import frc.robot.subsystems.LtClimber;
import frc.robot.subsystems.RtClimber;
import frc.robot.commands.ltClimb;
import frc.robot.commands.rtClimb;
import frc.robot.ArmCommands.FeedShooter;
import frc.robot.ArmCommands.ScoreAmp;
import frc.robot.ArmCommands.FloorIntake;




/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	private final Field2d field = new Field2d(); //  a representation of the field

	// The robot's subsystems
	private final Drivetrain m_robotDrive = new Drivetrain();
	private final CANLauncher m_launcher = new CANLauncher();
	//private final ArmSubsystem m_robotArm = new ArmSubsystem();
	private final Grabber m_grabber = new Grabber();
	private final RtClimber m_rtclimber = new RtClimber();
	private final LtClimber m_ltclimber = new LtClimber();
	public static Arm m_Arm= new Arm();

	// The driver's controller
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

	private final CommandXboxController m_operatorController =
	new CommandXboxController(OIConstants.kOperatorControllerPort);

	//public static RobotStates robotState;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		m_robotDrive.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			// We are inverting LeftY because Xbox controllers return negative values when we push forward.
			// We are inverting LeftX because we want a positive value when we pull to the left. Xbox controllers return positive values when you pull to the right by default.
			// We are also inverting RightX because we want a positive value when we pull to the left (CCW is positive in mathematics).
			new RunCommand(
				() -> m_robotDrive.drive(
					-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
					true, true),
				m_robotDrive));
	}
	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {

	
		//new JoystickButton(m_driverController, Button.kX.value)
			//.whileTrue(new RunCommand(
				//() -> m_robotDrive.setX(),
				//m_robotDrive));

		new JoystickButton(m_driverController, Button.kY.value)
			.onTrue(new InstantCommand(
				() -> m_robotDrive.resetEncoders(),
				m_robotDrive).ignoringDisable(true));

		new JoystickButton(m_driverController, Button.kA.value)
			.onTrue(new InstantCommand(
				() -> m_robotDrive.zeroHeading(),
				m_robotDrive).ignoringDisable(true)); 
				
		//new JoystickButton(m_driverController, Button.kB.value)
			//.onTrue(getAutonomousCommand());
	
	new JoystickButton (m_driverController, Button.kB.value).whileTrue(new rtClimb(m_rtclimber));
	new JoystickButton (m_driverController,Button.kRightBumper.value).whileTrue(m_rtclimber.getrtdownCommand());
	new JoystickButton (m_driverController, Button.kLeftBumper.value).whileTrue(new ltClimb(m_ltclimber));
	new JoystickButton (m_driverController,Button.kX.value).whileTrue(m_ltclimber.getltdownCommand());
	//m_driverController.povDown().onTrue(m_ltclimber.getltdownCommand());
	//m_driverController.getRightTrigger().whileTrue(m_rtclimber.getrtdownCommand());



   m_operatorController.x().whileTrue(new Grab(m_grabber).handleInterrupt(() -> m_grabber.stop()));
   m_operatorController.rightBumper().whileTrue(m_grabber.getEjectCommand());
   m_operatorController.leftBumper().whileTrue(m_launcher.getIntakeCommand());
   m_operatorController.b().onTrue(new FeedShooter());
   m_operatorController.y().onTrue(new FloorIntake());
   m_operatorController.povUp().onTrue(new ScoreAmp());

	/*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    m_operatorController
        .a()
        .whileTrue(new PrepareLaunch(m_launcher)
                .withTimeout(LauncherConstants.kLauncherDelay)
                .andThen(new LaunchNote(m_launcher)).handleInterrupt(() -> m_launcher.stop()));

			
}
	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.kMaxSpeedMetersPerSecond,
			AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.kDriveKinematics);

		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, new Rotation2d(0)),
			config);

		var thetaController = new ProfiledPIDController(
			AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
			
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			exampleTrajectory,
			m_robotDrive::getPose, // Functional interface to feed supplier
			DrivetrainConstants.kDriveKinematics,

			// Position controllers
			new PIDController(AutoConstants.kPXController, 0, 0),
			new PIDController(AutoConstants.kPYController, 0, 0),
			thetaController,
			m_robotDrive::setModuleStates,
			m_robotDrive);

		// Reset odometry to the starting pose of the trajectory.
		m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose()); // WARNING: https://github.com/REVrobotics/MAXSwerve-Java-Template/issues/13

		field.getObject("exampleTrajectory").setTrajectory(exampleTrajectory);

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
	}

	public Field2d getField()
	{
		return field;
	}

	public Drivetrain getDrive()
	{
		return m_robotDrive;
	}
}