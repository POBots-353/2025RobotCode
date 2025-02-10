// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

    public class SystemIdentification {
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new Mechanism(null, null, null)
    );
            public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
        }

        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
        }

}
