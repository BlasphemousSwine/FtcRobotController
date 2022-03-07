package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain {

    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    final DcMotor left;
    final DcMotor right;
    final DcMotor horizontal;

    private final BNO055IMU imu;

    Orientation angles;
    BNO055IMU.Parameters imuParameters;

    public double[] position = new double[3];
    public double[] setPosition = new double[3];

    public double[] setVelocity = new double[3];

    double xPosition = 0;
    double yPosition = 0;
    double rPosition = 0;

    final double xkp = 0.7;
    final double xki = 0.00;
    final double xkd = 20;
    double xErrSum = 0;
    double xLastTime = 0;
    double xLastErr = 0;

    final double ykp = 0.7;
    final double yki = 0.00;
    final double ykd = 20;
    double yErrSum = 0;
    double yLastTime = 0;
    double yLastErr = 0;

    final double rkp = 0.07;
    final double rki = 0.000;
    final double rkd = 0.7;
    double rErrSum = 0;
    double rLastTime = 0;
    double rLastErr = 0;

    double prevLeftPos = 0;
    double prevRightPos = 0;
    double prevHorizontalPos = 0;

    double lastFLPos;
    double lastFRPos;
    double lastBLPos;
    double lastBRPos;

    final double xConst = 1500;
    final double yConst = 1000;
//    final double rConst = 9.4

    final double trackWidth = 10;
    final double forwardOffset = 10;

    boolean useOdo = false;

    private final ElapsedTime runtime = new ElapsedTime();

    public Drivetrain(HardwareMap hardwareMap) {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        setPosition[0] = Double.NaN;
        setPosition[1] = Double.NaN;
        setPosition[2] = Double.NaN;

        if(useOdo) {
            left = hardwareMap.dcMotor.get("liftLeft");
            right = hardwareMap.dcMotor.get("liftRight");
            horizontal = hardwareMap.dcMotor.get("intake");
        }
        else {
            left = hardwareMap.dcMotor.get("fl");
            right = hardwareMap.dcMotor.get("fl");
            horizontal = hardwareMap.dcMotor.get("fl");
        }

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.loggingEnabled = false;
        initializeIMU();

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

        double radians = Math.toRadians(position[2]);

        double yPower = -(velCalc[1] * Math.cos(radians) - velCalc[0] * Math.sin(radians));
        double xPower = velCalc[1] * Math.sin(radians) + velCalc[0] * Math.cos(radians);

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

        if (Double.isNaN(setPosition[0])) {
            xVel = setVelocity[0];
        } else {
            xVel = xPosPID(setPosition[0]) + setVelocity[0];
        }

        if (Double.isNaN(setPosition[1])) {
            yVel = setVelocity[1];
        } else {
            yVel = yPosPID(setPosition[1]) + setVelocity[1];

        }
        if (Double.isNaN(setPosition[2])) {
            rPower = setVelocity[2];
        } else {
            rPower = - rPosPID(setPosition[2]) + setVelocity[2];
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

        Output = - (xkp * error + xki * xErrSum + xkd * dErr);

        xLastErr = error;
        xLastTime = now;

        if(Math.abs(error) <= 0.05) {
            return 0;
        }
        else {
            return Output;
        }

    }

    private double yPosPID(double setY) {

        double Output;

        double now = runtime.milliseconds();
        double timeChange = now - yLastTime;

        double error = position[1] - setY;
        yErrSum += (error * timeChange);
        double dErr = (error - yLastErr) / timeChange;

        Output = - (ykp * error + yki * yErrSum + ykd * dErr);

        yLastErr = error;
        yLastTime = now;

        if(Math.abs(error) <= 0.05) {
            return 0;
        }
        else {
            return Output;
        }

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

        if(Math.abs(error) <= 1) {
            return 0;
        }
        else {
            return Output;
        }

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
//            phi = (((dBLPos + dFLPos) - (dFRPos + dBRPos)) / 4) / rConst;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            lastFLPos = currentFLPos;
            lastFRPos = currentFRPos;
            lastBLPos = currentBLPos;
            lastBRPos = currentBRPos;

        }

        double radians = Math.toRadians(rPosition);

        double dXField = - (dXRobot * Math.cos(radians) - dYRobot * Math.sin(radians));
        double dYField = - (dXRobot * Math.sin(radians) + dYRobot * Math.cos(radians));

        xPosition += dXField;
        yPosition += dYField;
//        rPosition += phi;
        rPosition = -angles.firstAngle;

        position[0] = xPosition;
        position[1] = yPosition;
        position[2] = rPosition;

    }

//    public void setUseOdo(boolean setUseOdo) {
//
//        useOdo = setUseOdo
//
//    }

    public boolean checkPosition() {

        boolean xCheck = Math.abs(setPosition[0] - position[0]) <= 0.2;
        boolean yCheck = Math.abs(setPosition[1] - position[1]) <= 0.2;
        boolean rCheck = Math.abs(setPosition[2] - position[2]) <= 10;

        return(xCheck && yCheck && rCheck);

    }

    public void initializeIMU() {
        imu.initialize(imuParameters);
    }

}
