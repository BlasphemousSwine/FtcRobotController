package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    enum state {
        INTAKE, LOW, MID, HIGH, NONE
    }

    private state currentState = state.NONE;
    private state setState;

    public Lift(HardwareMap hardwareMap) {

    }

}
