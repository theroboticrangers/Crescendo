// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class DrivetrainConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double kMaxSpeedMetersPerSecond = 4.0; //4.42; //4.8;
		public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

		public static final double kDirectionSlewRate = 1.2; // radians per second
		public static final double kMagnitudeSlewRate = 1.8; // 2.0; //1.8; // percent per second (1 = 100%)
		public static final double kRotationalSlewRate = 2.0; // 20.0; //2.0; // percent per second (1 = 100%)

		// Chassis configuration
		public static final double kTrackWidth = Units.inchesToMeters(21.75);
		
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = Units.inchesToMeters(26.75);
		
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = 0; // -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0; // 0;
		public static final double kBackLeftChassisAngularOffset = 0; // Math.PI;
		public static final double kBackRightChassisAngularOffset = 0; // Math.PI / 2;

		// SPARK MAX CAN IDs
		public static final int kFrontLeftDrivingCanId = 6;
		public static final int kRearLeftDrivingCanId = 4;
		public static final int kFrontRightDrivingCanId = 8;
		public static final int kRearRightDrivingCanId = 2;

		public static final int kFrontLeftTurningCanId = 5;
		public static final int kRearLeftTurningCanId = 3;
		public static final int kFrontRightTurningCanId = 7;
		public static final int kRearRightTurningCanId = 1;

		 // SPARK MAX Absolute encoders
		public static final int kFrontLeftTurningAnalogPort = 2;
		public static final int kRearLeftTurningAnalogPort = 3;
		public static final int kFrontRightTurningAnalogPort = 0;
		public static final int kRearRightTurningAnalogPort = 1;

		public static final boolean kGyroReversed = false;
	}

	public static final class SwerveModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
		// This changes the drive speed of the module (a pinion gear with more teeth will result in a
		// robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14;

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		public static final boolean kTurningEncoderInverted = true;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
		public static final double kWheelDiameterMeters = 0.1016;
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
		public static final double kDrivingMotorReduction = (45.0 * 17 * 50) / (kDrivingMotorPinionTeeth * 15 * 27);
		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
				/ kDrivingMotorReduction;

		public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction; // meters
		public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction) / 60.0; // meters per second

		public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0; // ratio between internal relative encoder and Through Bore (or Thrifty in our case) absolute encoder - 150.0 / 7.0

		public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / TURNING_MOTOR_REDUCTION ; // radians - 
		public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / TURNING_MOTOR_REDUCTION / 60.0; // radians per second

		public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
		public static final double kTurningEncoderPositionPIDMaxInput = (2 * Math.PI); // radians

		public static final double kDrivingP = 0.04;
		public static final double kDrivingI = 0;
		public static final double kDrivingD = 0;
		public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
		public static final double kDrivingMinOutput = -1;
		public static final double kDrivingMaxOutput = 1;

		public static final double kTurningP = 1.0; //1.0; // 1.0 might be a bit too much - reduce a bit if needed
		public static final double kTurningI = 0;
		public static final double kTurningD = 0;
		public static final double kTurningFF = 0;
		public static final double kTurningMinOutput = -1;
		public static final double kTurningMaxOutput = 1;

		//public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
		//public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

		public static final com.revrobotics.CANSparkBase.IdleMode kDrivingMotorIdleMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;
		public static final com.revrobotics.CANSparkBase.IdleMode kTurningMotorIdleMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;

		public static final int kDrivingMotorCurrentLimit = 40; //50; // amps
		public static final int kTurningMotorCurrentLimit = 20; // amps
	}

	public static final class OIConstants {
		public static final int kOperatorControllerPort = 0;
		public static final int kDriverControllerPort = 2;
		public static final double kDriveDeadband = 0.05; //0.05;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3.0; //4.42; //3.0;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

		public static final double kPXController = 1;
		public static final double kPYController = 1;
		public static final double kPThetaController = 1;

		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 5676;
	}

	public static class LauncherConstants {
		// PWM ports/CAN IDs for motor controllers
		public static final int kFeederID = 15;
		public static final int kLauncherID = 12;
	
		// Current limit for launcher and feed wheels
		public static final int kLauncherCurrentLimit = 80;
		public static final int kFeedCurrentLimit = 80;
	
		// Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
		// in reverse
		public static final double kLauncherSpeed = 1;
		public static final double kLaunchFeederSpeed = 1;
		public static final double kIntakeLauncherSpeed = -.4;
		public static final double kIntakeFeederSpeed = -.2;
	
		public static final double kLauncherDelay = .1;
	}

public static final class ArmConstants {

	//Motors
	public static final int ArmID = 10;

    //Speeds of Arm
	public static final double moveUpSpeed = 1;
	public static final double moveDownSpeed = -1;
	public static final double moveUpSpeedSlow = -.1;
	public static final double moveDownSpeedSlow = .1;
    //Positions of Arm
	public static final double FLOOR_POSITION = 0.01;
    public static final double FEED_SHOOTER = .57;
    public static final double SCORE_AMP = 0.3;
    
	}
public static class GrabberConstants{
	//grabber ID can number
		public static final int grabberID=11;
		public static final boolean kMotorInverted = true;
        public static final int kCurrentLimit = 80;
		public static final double EjectSpeed=-.8;
		public static final double grabspeed = .7;
		  }
public static class ClimberConstants{
		// CAN IDs for motor controllers
			public static final int krtclimberID = 14;
			public static final int kltclimberID = 13;
		// Speeds for going up and down
			public static final double krtclimberupSpeed = 1;
			public static final double krtclimberdownSpeed = -1;
			public static final double kltclimberupSpeed = 1;
			public static final double kltclimberdownSpeed = -1;
			public static final int krtclimberCurrentLimit = 80;
			public static final int kltclimberCurrentLimit = 80;
	}

	
}



