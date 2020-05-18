package com.hdu.paper2code;

public class TrackSegment {
    int id;  //属于哪个传感器
    int interval; //属于哪个阶段
    double start_x;
    double start_y;
    double end_x;
    double end_y;

    public TrackSegment(int id, int interval, double start_x, double start_y, double end_x, double end_y) {
        this.id = id;
        this.interval = interval;
        this.start_x = start_x;
        this.start_y = start_y;
        this.end_x = end_x;
        this.end_y = end_y;
    }

    public String toString(){
        return "传感器"+id+"的第"+interval+"阶段"+", 起始("+start_x+", "+start_y+")，终止("+end_x+", "+end_y+") ";
    }
}
