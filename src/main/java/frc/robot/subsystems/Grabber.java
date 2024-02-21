// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GrabberConstants.*;

public class Grabber extends SubsystemBase {

  private CANSparkMax grabber_motor;
  private RelativeEncoder grabber_encoder;

/**
   * Creates a new Grabber Subsystem.
   */
  public Grabber() {
    // create a new SPARK MAX and configure it
    grabber_motor = new CANSparkMax(grabberID, MotorType.kBrushless);
    grabber_motor.setInverted(false);
    grabber_motor.setSmartCurrentLimit(kCurrentLimit);
    grabber_motor.setIdleMode(IdleMode.kCoast);

    grabber_encoder = grabber_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
  }

  public Command getEjectCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setGrabberWheel(EjectSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setGrabberWheel(double speed) {
    grabber_motor.set(speed);

  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    grabber_motor.set(0);
  }
    @Override
	public void periodic() {
    SmartDashboard.putNumber("Grabber Encoder Distance", grabber_encoder.getPosition());
    SmartDashboard.putNumber("Grabber Encoder Rate", grabber_encoder.getVelocity());
    SmartDashboard.putNumber("Distance Per Pulse,", grabber_encoder.getCountsPerRevolution());
  }

}