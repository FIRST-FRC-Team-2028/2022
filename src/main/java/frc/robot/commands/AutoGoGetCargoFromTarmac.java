// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pickup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGoGetCargoFromTarmac extends ParallelCommandGroup {
  /** Drives to cargo from the tarmac and use pickup to get cargo. */
  public AutoGoGetCargoFromTarmac(DriveSubsystem drive, Pickup pickup, Magazine magazine) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDriveToCargo(drive),
      new PickupTargets(pickup, magazine)
    );
  }
}
