package org.firstinspires.ftc.teamcode.opmodes.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class ServoTest extends LinearOpMode {

    Servo bucketServo;

    @Override
    public void runOpMode() throws InterruptedException {
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        bucketServo.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            bucketServo.setPosition(1);
        }
    }
}
