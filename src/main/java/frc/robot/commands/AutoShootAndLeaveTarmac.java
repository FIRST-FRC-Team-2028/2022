// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoShootAndLeaveTarmac extends SequentialCommandGroup {
  /** Shoot the starting cargo and 
   *  leave the tarmac using the AutoLeaveTarmac and Autoshoot commands
   */
  public AutoShootAndLeaveTarmac(Turret turret, Magazine magazine, DriveSubsystem drive) {
    double I_AM_NOT_DONE_HERE;
    addCommands(
      new AutoShoot(turret, magazine),
      // Will shoot happen instantaneously?
      new AutoLeaveTarmacTimed(drive)
    );
  }
}
