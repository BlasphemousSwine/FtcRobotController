package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {

    private final DcMotor motorFrontLeft;
    private final DcMotor motorFrontRight;
    private final DcMotor motorBackLeft;
    private final DcMotor motorBackRight;
    private final DcMotor motorLift;
    private final DcMotor motorIntake;
    private final Servo servoBox;
    private final CRServo servoDucks;
    private final BNO055IMU imu;

    Orientation angles;
    BNO055IMU.Parameters imuParameters;

    double xConst = 1000;
    double yConst = 1000;
//    double angleConst = 9.4

    double xField = 0;
    double yField = 0;
    double angle = 0;

    double xComplete = 0;
    double yComplete = 0;
    double angleComplete = 0;

    double xDist = 1;
    double yDist = 1;
    double angleDist = 1;

    double xInput = 0;
    double yInput = 0;
    double rInput = 0;

    double liftLow = 1;
    double liftMid = 2;
    double liftHigh = 3;

    double lastFrontLeftPos;
    double lastFrontRightPos;
    double lastBackLeftPos;
    double lastBackRightPos;

    public Robot(HardwareMap hardwareMap) {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        servoBox = hardwareMap.servo.get("servoBox");
        servoDucks = hardwareMap.crservo.get("ducks");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.loggingEnabled = false;
        initializeIMU();

    }

    public void initializeIMU() {
        imu.initialize(imuParameters);
    }

}