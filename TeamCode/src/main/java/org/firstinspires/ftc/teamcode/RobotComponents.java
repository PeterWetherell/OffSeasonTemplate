package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class RobotComponents {
    public ArrayList<Component> components;
    double wheelWidth;
    double wheelDiameter;
    double robotLength;
    double robotWidth;
    Point[] encoderPos;
    boolean drawnOdo = false;
    int componentNum = 0;
    int odoStart = 0;
    String bodyColor;
    public RobotComponents(boolean useOdometry){
        this(useOdometry,"#707070");
    }
    public RobotComponents(boolean useOdometry, String color){
        bodyColor = color;
        wheelWidth = 1.49606;
        wheelDiameter = 3.77953;
        robotLength = 15.5; robotLength ++;
        robotWidth = 12.5; robotWidth ++;
        components = new ArrayList<Component>();
        Localizer l = new Localizer();
        encoderPos = new Point[l.encoders.length];
        for (int i = 0; i < encoderPos.length; i ++){
            encoderPos[i] = new Point(l.encoders[i].x,l.encoders[i].y);
        }
        robotBody();
        robotWheels();
        robotDirectionIndicator();
        if (useOdometry) {
            robotOdo();
        }
    }
    public void robotOdo(){
        drawnOdo = true;
        odoStart = componentNum;
        for (int i = 0; i < encoderPos.length; i ++){
            Component odoPod = new Component(0.1);
            odoPod.color = bodyColor;
            odoPod.p.add(new Point(encoderPos[i].x,encoderPos[i].y));
            components.add(odoPod);
        }
    }
    public void setOdoColor(boolean isKnownPos){
        if (drawnOdo) {
            String color = bodyColor;
            if (!isKnownPos) {
                color = "#ff0000";
            }
            for (int i = 0; i < encoderPos.length; i++) {
                components.get(i + odoStart).color = color;
            }
        }
    }
    public void robotDirectionIndicator() {
        Component direction = new Component();
        direction.color = bodyColor;
        direction.p.add(new Point(0,0));
        direction.p.add(new Point(robotLength/2.0,0));
        components.add(direction); // 5
        componentNum ++;
    }
    public void robotBody() {
        Component robotBody = new Component();
        robotBody.color = bodyColor;
        robotBody.p.add(new Point(robotLength/ 2.0,robotWidth/2.0));
        robotBody.p.add(new Point(robotLength/ 2.0,robotWidth/-2.0));
        robotBody.p.add(new Point(robotLength/-2.0,robotWidth/-2.0));
        robotBody.p.add(new Point(robotLength/-2.0,robotWidth/2.0));
        components.add(robotBody); // 0
        componentNum ++;
    }
    public void robotWheels(){
        for (int i = 0; i < 4; i ++) {
            Component wheel = new Component();
            wheel.color = bodyColor;
            wheel.lineRadius=1;
            double n = 1.0;
            double b = 1.0;
            if (i%2 == 0) { n = -1.0; }
            if (i < 2) { b = -1.0; }
            double x = robotLength/2.0 - 0.5;
            double y = robotWidth/2.0 - 0.5;
            wheel.p.add(new Point((x)*b,(y)*n));
            wheel.p.add(new Point((x)*b,(y-wheelWidth)*n));
            wheel.p.add(new Point((x-wheelDiameter)*b,(y-wheelWidth)*n));
            wheel.p.add(new Point((x-wheelDiameter)*b,(y)*n));
            components.add(wheel); // 1, 2, 3, 4 respectively
            componentNum ++;
        }
    }
}
