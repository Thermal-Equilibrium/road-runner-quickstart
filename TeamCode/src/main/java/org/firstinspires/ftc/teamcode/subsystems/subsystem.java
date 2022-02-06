package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface subsystem {

    void init(HardwareMap hwmap);

    void initNoReset(HardwareMap hwmap);

    void update();

    Object subsystemState();

}

