// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ArmCommands;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.SetArmSpeed;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FloorIntake extends SequentialCommandGroup {
  /** Creates a new FloorIntake Command to bring the arm to the floor for intaking the note */
  public FloorIntake() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new SequentialCommandGroup(
      new SetArmPosition(ArmConstants.FLOOR_POSITION),
      new SetArmSpeed(ArmConstants.moveDownSpeed)));
  }
}
