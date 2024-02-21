package frc.robot.ArmCommands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;


public class SetArmSpeed extends Command {
  private double speed;
  /** Creates a new setWristIntakeSpeed. */
  public SetArmSpeed(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Arm);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Arm.setArmSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Arm.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}