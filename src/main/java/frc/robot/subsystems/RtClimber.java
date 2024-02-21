// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ClimberConstants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class RtClimber extends SubsystemBase {

  private CANSparkMax rtclimber_motor;
  private RelativeEncoder rtclimber_encoder;

/**
   * Creates a new Right Climber Subsystem.
   */
  public RtClimber() {
    // create a new SPARK MAX and configure it
    rtclimber_motor = new CANSparkMax(ClimberConstants.krtclimberID, MotorType.kBrushless);
    rtclimber_motor.setInverted(false);
    rtclimber_motor.setSmartCurrentLimit(ClimberConstants.krtclimberCurrentLimit);
    rtclimber_motor.setIdleMode(IdleMode.kBrake);

    rtclimber_encoder = rtclimber_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
  }

  public Command getrtdownCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setrtClimberWheel(krtclimberdownSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setrtClimberWheel(double speed) {
    rtclimber_motor.set(speed);

  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    rtclimber_motor.set(0);
  }
    @Override
	public void periodic() {
    SmartDashboard.putNumber("Right Climber Distance", rtclimber_encoder.getPosition());
    SmartDashboard.putNumber("Right Climber Encoder Rate", rtclimber_encoder.getVelocity());
    SmartDashboard.putNumber("Distance Per Pulse,", rtclimber_encoder.getCountsPerRevolution());
    }

}