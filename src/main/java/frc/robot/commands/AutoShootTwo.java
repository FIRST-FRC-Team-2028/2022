// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootTwo extends SequentialCommandGroup {
  /** Shoots Initial cargo then gets more cargo then picks it up and shoots it */
  public AutoShootTwo(Turret turret, Magazine magazine, DriveSubsystem drive, Pickup pickup) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShootAndGetCargo(turret, magazine, drive, pickup),
      new CheckForCargo(pickup),
      new AutoShoot(turret, magazine)
    );
  }
}