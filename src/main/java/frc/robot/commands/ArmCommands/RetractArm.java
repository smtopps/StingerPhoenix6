// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pincher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractArm extends SequentialCommandGroup {
  /** Creates a new RetractArm. */
  public RetractArm(Pincher pincher, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> arm.setArmPIDValue(Constants.returnArmPGain), arm),
      new Release(pincher),
      new MoveExtensionToPosition(arm, () -> Constants.placeExtensionPosition),
      new MoveArmToPosition(arm, () -> Constants.pickupConeArmPosition),
      new MoveExtensionToPosition(arm, () -> Constants.pickupConeExtensionPosition)
    );
  }
}
