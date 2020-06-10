package frc.robot.subsystem.spinnyboi;

import edu.wpi.first.wpilibj.util.Color;

public class SpinnyBoiConstants {

    static final double SECONDS_PER_ARC = 0.71;
    // public static final double SPIN_NUM = 32768.0;

    static final double SPINNYBOI_FORWARD = 0.5;

    static final double SPINNIYBOI_BACKWARD = 0.5;

    static final Color yellow = new Color(255, 255, 0); // 0
    static final SpinnyBoiColor yellowIndex = new SpinnyBoiColor(yellow, 0, 2);

    static final Color red = new Color(255, 0, 0); // 1
    static final SpinnyBoiColor redIndex = new SpinnyBoiColor(red, 1, 3);


    static final Color green = new Color(0, 255, 0); // 2
    static final SpinnyBoiColor greenIndex = new SpinnyBoiColor(green, 2, 0);

    static final Color blue = new Color(0, 255, 255); // 3
    static final SpinnyBoiColor blueIndex = new SpinnyBoiColor(blue, 3, 1);

}
