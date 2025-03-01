package org.firstinspires.ftc.teamcode.hardware;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.movement.Pose;
public class ValueStorage {
    public enum Side {
        RED, BLUE
    }
    public static Telemetry telemetry;
    public static Side lastSide = Side.BLUE;
    public static Pose lastPose = new Pose(0, 0, 0);
    public static double voltage = 0;
}
