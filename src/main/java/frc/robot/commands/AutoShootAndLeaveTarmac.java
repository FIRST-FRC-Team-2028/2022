// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoShootAndLeaveTarmac extends SequentialCommandGroup {
  /** Shoot the starting cargo and 
   *  leave the tarmac using the AutoLeaveTarmac and Autoshoot commands
   */
  public AutoShootAndLeaveTarmac(Shooter turret, Magazine magazine, DriveSubsystem drive, Pickup pickup)  {
    addCommands(
      new AutoShoot(turret, magazine, pickup),

      new AutoShootWait(Constants.TURRET_TIME_TO_SHOOT),
      //new ZeroTurret(turret),
      // Will shoot happen instantaneously?
      new AutoLeaveTarmacTimed(drive)
    );
  }
}
