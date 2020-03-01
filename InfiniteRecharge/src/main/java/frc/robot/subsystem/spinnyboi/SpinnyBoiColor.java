package frc.robot.subsystem.spinnyboi;

import edu.wpi.first.wpilibj.util.Color;

public class SpinnyBoiColor {
    final Color color;
    final int index;
    final int spinnyBoiSensorTarget;

    public SpinnyBoiColor(Color color, int index, int spinnyBoiSensorTarget) {
        this.color = color;
        this.index = index;
        this.spinnyBoiSensorTarget = spinnyBoiSensorTarget;
    }
}