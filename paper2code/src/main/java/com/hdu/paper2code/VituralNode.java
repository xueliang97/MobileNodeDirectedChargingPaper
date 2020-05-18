package com.hdu.paper2code;

public class VituralNode {
    int id;//哪个传感器的虚拟节点
    double x_coordinate;
    double y_coordinate;

    public VituralNode(int id,double x_coordinate, double y_coordinate) {
        this.x_coordinate = x_coordinate;
        this.y_coordinate = y_coordinate;
        this.id = id;
    }

    @Override
    public String toString() {
        return "id为"+id+"("+x_coordinate+","+y_coordinate+")、";
    }
}
