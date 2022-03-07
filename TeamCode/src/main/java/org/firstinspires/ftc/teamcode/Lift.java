package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    final DcMotor LeftLift;
    final Servo LeftAngle;
    final Servo LeftV4B;
    final Servo LeftBox;
    final DcMotor RightLift;
    final Servo RightAngle;
    final Servo RightV4B;
    final Servo RightBox;
    final DcMotor IntakeMotor;
    final DcMotor Ducks;

    private double duck = 1;

    public Lift(HardwareMap hardwareMap) {

        LeftLift = hardwareMap.dcMotor.get("LeftLift");
        LeftAngle = hardwareMap.servo.get("LeftAngle");
        LeftV4B = hardwareMap.servo.get("LeftV4B");
        LeftBox = hardwareMap.servo.get("LeftBox");
        RightLift = hardwareMap.dcMotor.get("RightLift");
        RightAngle = hardwareMap.servo.get("RightAngle");
        RightV4B = hardwareMap.servo.get("RightV4B");
        RightBox = hardwareMap.servo.get("RightBox");
        IntakeMotor = hardwareMap.dcMotor.get("Intake");
        Ducks = hardwareMap.dcMotor.get("Ducks");

        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Ducks.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void drop() {

        IntakeMotor.setPower(0);
        LeftAngle.setPosition(1);
        RightAngle.setPosition(0);

    }

    public void capDown() {

        LeftV4B.setPosition(0.1);
        RightV4B.setPosition(0.9);

    }

    public void capUp() {

        LeftAngle.setPosition(0.7);
        RightAngle.setPosition(0.3);

    }

    public void extend() {

        drop();
        LeftV4B.setPosition(0.38);
        RightV4B.setPosition(0.62);

    }

    public void extendShared() {

        LeftAngle.setPosition(0.7);
        RightAngle.setPosition(0.3);

        IntakeMotor.setPower(0);
        LeftV4B.setPosition(0);
        RightV4B.setPosition(1);


    }

    public void retract() {

        drop();

        LeftV4B.setPosition(1);
        RightV4B.setPosition(0);
        LeftBox.setPosition(0);
        RightBox.setPosition(0);

    }

    public void dump() {

        LeftBox.setPosition(0.1);
        RightBox.setPosition(0.1);

    }

    public void unDump() {

        LeftBox.setPosition(0.4);
        RightBox.setPosition(0.4);

    }

    public void intake() {

        LeftAngle.setPosition(0.5);
        RightAngle.setPosition(0.5);
        LeftBox.setPosition(0);
        RightBox.setPosition(0);
        IntakeMotor.setPower(-1);

    }

    public void raise() {
        LeftAngle.setPosition(0.5);
        RightAngle.setPosition(0.5);
        LeftBox.setPosition(0);
        RightBox.setPosition(0);
    }

    public void backwards() {

        IntakeMotor.setPower(1);

    }

    public void DucksStart() {

        duck = Ducks.getCurrentPosition();

    }

    public void DucksUpdate(double duckSide) {

        if (Math.abs(Ducks.getCurrentPosition() - duck) <= 280) {
            Ducks.setPower(0.25 * duckSide);
        }
        else if (Math.abs(Ducks.getCurrentPosition() - duck) <= 520) {
            Ducks.setPower(0.35 * duckSide);
        }
        else if (Math.abs(Ducks.getCurrentPosition() - duck) <= 1200) {
            Ducks.setPower(0.5 * duckSide);
        }
        else {
            Ducks.setPower(0);
        }

    }

}
