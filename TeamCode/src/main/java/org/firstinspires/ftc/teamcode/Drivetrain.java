package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {

    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;
    private final DcMotor left;
    private final DcMotor right;
    private final DcMotor horizontal;

    public double[] position = new double[3];
    public double[] setPosition = new double[3];

    public double[] setVelocity;

    double xPosition = 0;
    double yPosition = 0;
    double rPosition = 0;

    final double xkp = 1;
    final double xki = 1;
    final double xkd = 1;
    double xErrSum = 0;
    double xLastTime = 0;
    double xLastErr = 0;

    final double ykp = 1;
    final double yki = 1;
    final double ykd = 1;
    double yErrSum = 0;
    double yLastTime = 0;
    double yLastErr = 0;

    final double rkp = 1;
    final double rki = 1;
    final double rkd = 1;
    double rErrSum = 0;
    double rLastTime = 0;
    double rLastErr = 0;

    double prevLeftPos;
    double prevRightPos;
    double prevHorizontalPos;

    final double trackWidth = 10;
    final double forwardOffset = 10;

    private final ElapsedTime runtime = new ElapsedTime();

    public Drivetrain(HardwareMap hardwareMap) {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        left = hardwareMap.dcMotor.get("liftLeft");
        right = hardwareMap.dcMotor.get("liftRight");
        horizontal = hardwareMap.dcMotor.get("intake");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void update() {

        getPosition();

        double xVel = xPosPID(setPosition[0]) + setVelocity[0];
        double yVel = yPosPID(setPosition[1]) + setVelocity[1];
        double rPower = rPosPID(setPosition[2]) + setVelocity[2];

        double yPower = yVel * Math.cos(position[2]) - xVel * Math.sin(position[2]);
        double xPower = yVel * Math.sin(position[2]) + xVel * Math.cos(position[2]);

        double scale = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(rPower), 1);
        double flPower = (yPower + xPower + rPower) / scale;
        double blPower = (yPower - xPower + rPower) / scale;
        double frPower = (yPower - xPower - rPower) / scale;
        double brPower = (yPower + xPower - rPower) / scale;

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);

    }

    private double xPosPID(double setX) {

        double now = runtime.milliseconds();
        double timeChange = now - xLastTime;

        double error = position[0] - setX;
        xErrSum += (error * timeChange);
        double dErr = (error - xLastErr) / timeChange;

        double Output = xkp * error + xki * xErrSum + xkd *dErr;

        xLastErr = error;
        xLastTime = now;

        return Output;

    }

    private double yPosPID(double setX) {

        double now = runtime.milliseconds();
        double timeChange = now - yLastTime;

        double error = position[1] - setX;
        yErrSum += (error * timeChange);
        double dErr = (error - yLastErr) / timeChange;

        double Output = ykp * error + yki * yErrSum + ykd *dErr;

        yLastErr = error;
        yLastTime = now;

        return Output;

    }

    private double rPosPID(double setX) {

        double now = runtime.milliseconds();
        double timeChange = now - rLastTime;

        double error = position[2] - setX;
        rErrSum += (error * timeChange);
        double dErr = (error - rLastErr) / timeChange;

        double Output = rkp * error + rki * rErrSum + rkd *dErr;

        rLastErr = error;
        rLastTime = now;

        return Output;

    }

    public void getPosition() {

        double currentLeftPos = left.getCurrentPosition();
        double currentRightPos = right.getCurrentPosition();
        double currentHorizontalPos = horizontal.getCurrentPosition();

        double dLeft = currentLeftPos - prevLeftPos;
        double dRight = currentRightPos - prevRightPos;
        double dHorizontal = currentHorizontalPos - prevHorizontalPos;

        double phi = (dLeft - dRight) / trackWidth;
        double dYRobot = (dLeft + dRight) / 2;
        double dXRobot = dHorizontal - forwardOffset * phi;

        double dXField = dXRobot * Math.cos(rPosition) - dYRobot * Math.sin(rPosition);
        double dYField = dXRobot * Math.sin(rPosition) + dYRobot * Math.cos(rPosition);

        xPosition += dXField;
        yPosition += dYField;
        rPosition += phi;

        prevHorizontalPos = currentHorizontalPos;
        prevLeftPos = currentLeftPos;
        prevRightPos = currentRightPos;

        position[0] = xPosition;
        position[1] = yPosition;
        position[2] = rPosition;

    }

}
