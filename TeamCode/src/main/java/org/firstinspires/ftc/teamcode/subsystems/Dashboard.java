package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;

public class Dashboard implements subsystem {

    ElapsedTime dashboardTimer = new ElapsedTime();

    public static TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init(HardwareMap hwmap) {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    @Override
    public void initNoReset(HardwareMap hwmap) {
        init(hwmap);
    }

    @Override
    public void update() {

        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        packet.put("Loop time",dashboardTimer.milliseconds());
        dashboardTimer.reset();

    }

    @Override
    public Vector3D subsystemState() {
        return null;
    }

    public void startCameraStream(CameraStreamSource source, int maxFps) {
        dashboard.startCameraStream(source,maxFps);
    }


}
