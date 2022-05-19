package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Module {

    private final DcMotor motor1;
    private final DcMotor motor2;

    public Module(HardwareMap hardwareMap, int ID) {

        motor1 = hardwareMap.dcMotor.get(ID + "1");
        motor2 = hardwareMap.dcMotor.get(ID + "2");

    }

    public void Update() {



    }

}
