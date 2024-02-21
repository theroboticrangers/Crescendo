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

public class LtClimber extends SubsystemBase {

  private CANSparkMax ltclimber_motor;
  private RelativeEncoder ltclimber_encoder;

/**
   * Creates a new Grabber Subsystem.
   */
  public LtClimber() {
    // create a new SPARK MAX and configure it
    ltclimber_motor = new CANSparkMax(ClimberConstants.kltclimberID, MotorType.kBrushless);
    ltclimber_motor.setInverted(false);
    ltclimber_motor.setSmartCurrentLimit(ClimberConstants.kltclimberCurrentLimit);
    ltclimber_motor.setIdleMode(IdleMode.kBrake);

    ltclimber_encoder = ltclimber_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
  }

  public Command getltdownCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setltClimberWheel(kltclimberdownSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setltClimberWheel(double speed) {
    ltclimber_motor.set(speed);

  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    ltclimber_motor.set(0);
  }
    @Override
	public void periodic() {
    SmartDashboard.putNumber("Left Climber Encoder Distance", ltclimber_encoder.getPosition());
    SmartDashboard.putNumber("Left Climber Encoder Rate", ltclimber_encoder.getVelocity());
    SmartDashboard.putNumber("Distance Per Pulse,", ltclimber_encoder.getCountsPerRevolution());
  }

}