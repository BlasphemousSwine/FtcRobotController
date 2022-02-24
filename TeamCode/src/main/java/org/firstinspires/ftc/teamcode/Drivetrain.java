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

    double lastFLPos;
    double lastFRPos;
    double lastBLPos;
    double lastBRPos;

    final double xConst = 1000;
    final double yConst = 1000;
    final double rConst = 9.4;

    final double trackWidth = 10;
    final double forwardOffset = 10;

    boolean useOdo = false;

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
        setPower();

    }

    private void setPower() {

        double[] powerCalc = powerCalc();

        fl.setPower(powerCalc[3]);
        bl.setPower(powerCalc[0]);
        fr.setPower(powerCalc[1]);
        br.setPower(powerCalc[2]);

    }

    private double[] powerCalc() {

        double[] velCalc = velCalc();

        double yPower = velCalc[1] * Math.cos(position[2]) - velCalc[0] * Math.sin(position[2]);
        double xPower = velCalc[1] * Math.sin(position[2]) + velCalc[0] * Math.cos(position[2]);

        double scale = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(velCalc[2]), 1);
        double blPower = (yPower - xPower + velCalc[2]) / scale;
        double frPower = (yPower - xPower - velCalc[2]) / scale;
        double brPower = (yPower + xPower - velCalc[2]) / scale;
        double flPower = (yPower + xPower + velCalc[2]) / scale;

        double[] powerCalc = new double[4];

        powerCalc[0] = blPower;
        powerCalc[1] = frPower;
        powerCalc[2] = brPower;
        powerCalc[3] = flPower;

        return powerCalc;

    }

    private double[] velCalc()

    {

        double xVel, yVel, rPower;

        if (Double.isNaN(xPosPID(setPosition[0]))) {
            xVel = setVelocity[0];
        } else {
            xVel = xPosPID(setPosition[0]) + setVelocity[0];
        }

        if (Double.isNaN(yPosPID(setPosition[1]))) {
            yVel = setVelocity[1];
        } else {
            yVel = yPosPID(setPosition[1]) + setVelocity[1];

        }
        if (Double.isNaN(rPosPID(setPosition[2]))) {
            rPower = setVelocity[2];
        } else {
            rPower = rPosPID(setPosition[2]) + setVelocity[2];
        }

        double[] velCalc = new double[3];

        velCalc[0] = xVel;
        velCalc[1] = yVel;
        velCalc[2] = rPower;

        return velCalc;

    }

    private double xPosPID(double setX) {

        double Output;

        double now = runtime.milliseconds();
        double timeChange = now - xLastTime;

        double error = position[0] - setX;
        xErrSum += (error * timeChange);
        double dErr = (error - xLastErr) / timeChange;

        Output = xkp * error + xki * xErrSum + xkd * dErr;

        xLastErr = error;
        xLastTime = now;

        return Output;

    }

    private double yPosPID(double setY) {

        double Output;

        double now = runtime.milliseconds();
        double timeChange = now - yLastTime;

        double error = position[1] - setY;
        yErrSum += (error * timeChange);
        double dErr = (error - yLastErr) / timeChange;

        Output = ykp * error + yki * yErrSum + ykd * dErr;

        yLastErr = error;
        yLastTime = now;

        return Output;

    }

    private double rPosPID(double setR) {

        double Output;

        double now = runtime.milliseconds();
        double timeChange = now - rLastTime;

        double error = position[2] - setR;
        rErrSum += (error * timeChange);
        double dErr = (error - rLastErr) / timeChange;

        Output = rkp * error + rki * rErrSum + rkd * dErr;

        rLastErr = error;
        rLastTime = now;

        return Output;

    }

    public void getPosition() {

        double phi, dYRobot, dXRobot;

        if(useOdo) {

            double currentLeftPos = left.getCurrentPosition();
            double currentRightPos = right.getCurrentPosition();
            double currentHorizontalPos = horizontal.getCurrentPosition();

            double dLeft = currentLeftPos - prevLeftPos;
            double dRight = currentRightPos - prevRightPos;
            double dHorizontal = currentHorizontalPos - prevHorizontalPos;

            phi = (dLeft - dRight) / trackWidth;
            dYRobot = (dLeft + dRight) / 2;
            dXRobot = dHorizontal - forwardOffset * phi;

            prevHorizontalPos = currentHorizontalPos;
            prevLeftPos = currentLeftPos;
            prevRightPos = currentRightPos;

        }

        else {

            double currentFLPos = fl.getCurrentPosition();
            double currentFRPos = fr.getCurrentPosition();
            double currentBLPos = bl.getCurrentPosition();
            double currentBRPos = br.getCurrentPosition();

            double dFLPos = currentFLPos - lastFLPos;
            double dFRPos = currentFRPos - lastFRPos;
            double dBLPos = currentBLPos - lastBLPos;
            double dBRPos = currentBRPos - lastBRPos;

            dXRobot = (((dFLPos + dBRPos) - (dFRPos + dBLPos)) / 4) / xConst;
            dYRobot = ((dFLPos + dFRPos + dBLPos + dBRPos) / 4) / yConst;
            phi = (((dBLPos + dFLPos) - (dFRPos + dBRPos)) / 4) / rConst;

            lastFLPos = currentFLPos;
            lastFRPos = currentFRPos;
            lastBLPos = currentBLPos;
            lastBRPos = currentBRPos;

        }

        double dXField = dXRobot * Math.cos(rPosition) - dYRobot * Math.sin(rPosition);
        double dYField = dXRobot * Math.sin(rPosition) + dYRobot * Math.cos(rPosition);

        xPosition += dXField;
        yPosition += dYField;
        rPosition += phi;

        position[0] = xPosition;
        position[1] = yPosition;
        position[2] = rPosition;

    }

    public void setUseOdo(boolean setUseOdo) {

        useOdo = setUseOdo;

    }

}
