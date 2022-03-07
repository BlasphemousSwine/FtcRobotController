package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "servoTest")
public class ServoTest extends LinearOpMode{

    @Override
    public void runOpMode() {

        Servo a = hardwareMap.servo.get("george0");
        Servo b = hardwareMap.servo.get("george1");
        Servo c = hardwareMap.servo.get("george2");
        Servo d = hardwareMap.servo.get("george3");

        waitForStart();

        if(!isStopRequested()) {
            while(opModeIsActive()) {
                if(gamepad1.a) {
                    a.setPosition(1);
                    b.setPosition(0);
                }
                if(gamepad1.b) {
                    a.setPosition(0.5);
                    b.setPosition(0.5);
                }
                if(gamepad1.y) {
                    a.setPosition(0);
                    b.setPosition(1);
                }
                if(gamepad1.dpad_down) {
                    c.setPosition(0);
                    d.setPosition(1);
                }
                if(gamepad1.dpad_right) {
                    c.setPosition(0.5);
                    d.setPosition(0.5);
                }
                if(gamepad1.dpad_up) {
                    c.setPosition(1);
                    d.setPosition(0);
                }
            }
        }

    }

}
