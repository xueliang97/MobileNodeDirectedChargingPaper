package com.hdu.paper2code;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class CandidatePosition {
    int id;
    double x_coordinate;
    double y_coordinate;
    double utility;
    double start_orientation; //右边的开始边
    double end_orientation;
    List<TrackSegment> trackSegmentsInRange; //范围内的轨迹,整个圆
    List<VituralNode> vituralNodeInRange; //范围内的虚拟节点，整个圆
    Map<Double,List<VituralNode>> orientation_VituralNodes; //不同朝向对应的范围内虚拟节点
    Map<Double,VituralNode> angle_vituralNode;//不同角度对应的虚拟节点

    public CandidatePosition(int id,double x_coordinate, double y_coordinate) {
        this.id = id;
        this.x_coordinate = x_coordinate;
        this.y_coordinate = y_coordinate;
        trackSegmentsInRange = new ArrayList<>();
        vituralNodeInRange = new ArrayList<>();
        orientation_VituralNodes = new TreeMap<>();
        angle_vituralNode = new TreeMap<>();
    }

    public String toString(){
        return "位置id为"+id+"，最佳方向："+start_orientation+"，效用"+utility;
    }
}
