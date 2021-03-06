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
public class Shoot extends SequentialCommandGroup {
  Pickup pickup;
  /** Shoot a Cargo into the Hub 
   *    assuming the turret is aimed,
   *    and there is cargo onboard
   *    set up the range
   *    fire the cargo
  */
  public Shoot(Magazine magazine, Shooter turret, Pickup pickup) {
    /* TODO: eventually
    double AM_I_DONE = 00000.;  // do we need to ensure there is something to shoot?
    if (pickup.numCargo() < 1) {
      return;
    }
    */
    addCommands(
      /**sets motor speed and elevation */
      new ShootInit(turret),
      /** moves ball from magazine to turret*/
      new ShootFire(magazine, pickup, turret)
    );
  }
}
