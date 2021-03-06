// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoShoot extends SequentialCommandGroup {
  /**  Shoot the starting cargo when the robot is on the tarmac
   *    Presuming the turret is facing the hub, (so there is no need to aim)
   *    set tarmac distance
   *    shoot
   */
  public AutoShoot(Shooter turret, Magazine magazine, Pickup pickup) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetTarmacDistance(turret),
      new Shoot(magazine, turret, pickup)
    );
  }
}
