package com.hdu.paper2code;

public class Util {
    public static double distance(double x1,double y1,double x2,double y2){
        return Math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }

    public static double angle(double x1,double y1,double x2,double y2){ //计算两点之间的角度,弧度
        double cosa = (x2-x1)/distance(x1, y1, x2, y2);
        double a = Math.acos(cosa);
        if(y2-y1<0)
            a = 2*MovableNode.PI-a;
        return a;
    }

    public static double calPr(double x1,double y1,double x2,double y2){
        double dis = distance(x1, y1, x2, y2);
        if (dis>MovableNode.D)
            return 0;
        return  MovableNode.alpha/Math.pow(dis+MovableNode.beta,2);
    }

    public static double calutility(double x1,double y1,double x2,double y2){
        double Pr = calPr(x1, y1, x2, y2);
        double t = MovableNode.deltal / MovableNode.VELOCITY;
        if(Pr>MovableNode.Pth)
            return t*MovableNode.Pth;
        return t*Pr;
    }
}
