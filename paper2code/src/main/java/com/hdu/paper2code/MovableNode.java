package com.hdu.paper2code;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.TreeMap;

public class MovableNode {

    public static final double PI = 3.14159;
    public static final double A = PI/2; //充电器的充电范围角度
    public static final double alpha = 100;
    public static final double beta = 0.5;   //Pr = alpha/(d+beta)^2
    public static final double Cu = 1;
    public static final int LENGTH = 50; //m
    public static final int WIDTH = 50;
    public static final int D = 8; //最大充电距离
    public static final double VELOCITY = 1;
    public static final int SENSORS_NUM = 15;
    public static final int CHARGERS_NUM = 20;
    public static final int CandidatePosition_NUM = 25;
    public static final int INTERVAL = 5;//移动过程分成的时间段，结束后回到起点
    public static final int INTERVAL_TIME = 10;//每段时间
    public static final double Pth = 2; //充电功率的最大值，达到该功率时效用变为常数
    public static final double deltal = 1;//划分最小小段的长度

    static List<List<TrackSegment>> sensorsTrack;//存放所有节点的轨迹
    static List<CandidatePosition> candidatePositionList;//所有的候选位置
    static List<CandidatePosition> usablePosition;//存放可用的位置
    static List<TrackSegment> allTrack;//所有的轨迹段

    public static double myPaperUtility=0,comparedPaperUtility=0,baselineUtility=0;

    public static void main(String[] args){
        init();
        if (calUsablePosition()<CHARGERS_NUM) {
            System.out.println("生成候选位置不符合条件，重来");
            return;
        }
        Collections.sort(usablePosition, new Comparator<CandidatePosition>() {
            @Override
            public int compare(CandidatePosition t1, CandidatePosition t2) {
                return t1.id-t2.id;
            }
        });
        myPaper();
        contrastAlgorithm();
        baselineAlgorithm();
        System.out.println("本算法："+myPaperUtility+" 对比论文算法："+comparedPaperUtility+" 基线算法："+baselineUtility);
    }

    public static void myPaper(){


        for(int i=1;i<=INTERVAL;i++){
            List<TrackSegment> trackPeriod = calTrackInPeriod(i);//当前时段场地内的轨迹
            List<VituralNode> currentVirtualNodeList=constructVirtualNode(trackPeriod);
            System.out.println("阶段"+i);
            for(CandidatePosition p:usablePosition){
                calOptionalOrientation(p,currentVirtualNodeList);
                calBestOrientation(p,i);
                myPaperUtility+= p.utility;
                System.out.println(p);
            }
        }
        System.out.println("总效用"+myPaperUtility);



    }

    public static void contrastAlgorithm(){
        System.out.println();
        System.out.println("对比前面论文的算法：");
        double sumUtility = 0;
        List<TrackSegment> trackPeriod = new ArrayList<>();
        for(List<TrackSegment> list:sensorsTrack){
            for(TrackSegment segment:list){
                trackPeriod.add(segment);
            }
        }
        List<VituralNode> currentVirtualNodeList=constructVirtualNode(trackPeriod);
        for(CandidatePosition p:usablePosition){
            calOptionalOrientation(p,currentVirtualNodeList);
            contrastAlgorithmChooseOrientation(p);
            comparedPaperUtility += p.utility;
            System.out.println(p);
        }
        System.out.println(comparedPaperUtility);
    }

    public static void baselineAlgorithm(){
        System.out.println();
        System.out.println("基线算法");
        double sumUtility = 0;

        for(int i=1;i<=INTERVAL;i++){
            List<TrackSegment> trackPeriod = calTrackInPeriod(i);//当前时段场地内的轨迹
            List<VituralNode> currentVirtualNodeList=constructVirtualNode(trackPeriod);
            System.out.println("阶段"+i);
            for(CandidatePosition p:usablePosition){
                blCalOptionalOrientation(p,currentVirtualNodeList);
                calBestOrientation(p,i);
                baselineUtility += p.utility;
                System.out.println(p);
            }
        }
        System.out.println("总效用"+baselineUtility);
    }



    public static void init(){
        sensorsTrack = new LinkedList<>();
        for (int i=0;i<SENSORS_NUM;i++)
            sensorsTrack.add(generateTrajectory(i));
        initCandidatePosition();
    }

    public static void initCandidatePosition(){
        candidatePositionList = new ArrayList<>();
        usablePosition = new ArrayList<>();
        for (int i=0;i<CandidatePosition_NUM;i++){
            CandidatePosition position = new CandidatePosition(i,Math.random()*LENGTH,Math.random()*LENGTH);
            candidatePositionList.add(position);
        }
    }

    public static List<TrackSegment> generateTrajectory(int id){
        List<TrackSegment> list = new ArrayList<>();
        double start_x = -1;
        double start_y = -1;
        while(start_x<0||start_x>LENGTH||start_y<0||start_y>WIDTH){
            start_x = LENGTH * Math.random();
            start_y = WIDTH * Math.random();
        }

        TrackSegment zero = new TrackSegment(id,0,0,0,start_x,start_y);//初始化节点
        list.add(zero);
        for(int i=1;i<=INTERVAL;i++){
            TrackSegment tracksegment = new TrackSegment(id,i,0,0,0,0);
            tracksegment.start_x = list.get(i-1).end_x;
            tracksegment.start_y = list.get(i-1).end_y;
            generateEquidistantNode(tracksegment);
            list.add(tracksegment);

        }
        list.remove(0);
        System.out.println("轨迹"+list);
        return list;
    }

    public static void generateEquidistantNode(TrackSegment t){//生成轨迹中下一个节点
        double distance = VELOCITY*INTERVAL_TIME;
        double K = Math.random()*5; //斜率的平方
        double x=-1,y=-1;
        while (x<0||x>LENGTH||y<0||y>WIDTH){
            x = t.start_x+(Math.random()>0.5 ? 1:-1)*(distance/Math.sqrt(1+K));  //x
            y = t.start_y+(Math.random()>0.5 ? 1:-1)*distance*Math.sqrt(K/(1+K));  //y
        }
        t.end_x = x;
        t.end_y = y;
    }

    public static List<TrackSegment> calTrackInPeriod(int period){//计算该时段中场地内的轨迹 1<=period<=INTERVAL
        List<TrackSegment> list = new ArrayList<>();
        for(List<TrackSegment> temp:sensorsTrack){
            for(int i=0;i<temp.size();i++){
                if(temp.get(i).interval == period)
                    list.add(temp.get(i));
            }
        }
        return list;
    }

    public static int calUsablePosition(){
        allTrack = new ArrayList<>();
        for(List<TrackSegment> t:sensorsTrack){
            for(TrackSegment segment:t){
                allTrack.add(segment);
            }
        }
        int count = 0;
        for(CandidatePosition position:candidatePositionList){
            for(TrackSegment trackSegment:allTrack){
                double startPointDis = Util.distance(trackSegment.start_x,trackSegment.start_y,position.x_coordinate,position.y_coordinate);
                double endPointDis = Util.distance(trackSegment.end_x,trackSegment.end_y,position.x_coordinate,position.y_coordinate);
                if(startPointDis<=D||endPointDis<=D){
                    position.trackSegmentsInRange.add(trackSegment);
                }
            }
        }

        for(CandidatePosition p:candidatePositionList) {
            p.utility = p.trackSegmentsInRange.size();
            if (p.utility>0)
                count++;
        }
        if (count<CHARGERS_NUM)
            return count;


        PriorityQueue<CandidatePosition> queue =  new PriorityQueue<>(CHARGERS_NUM, new Comparator<CandidatePosition>() {
            @Override
            public int compare(CandidatePosition candidatePosition, CandidatePosition t1) {
                return (int) (t1.utility-candidatePosition.utility);
            }
        });
        for (CandidatePosition p:candidatePositionList) {
            queue.add(p);
            p.utility = 0;
        }
        for (int i=0;i<CHARGERS_NUM;i++)
            usablePosition.add(queue.poll());

        return usablePosition.size();


    }

    public static List<VituralNode> constructVirtualNode(List<TrackSegment> trackPeriod){//生成当前时段的虚拟节点
        List<VituralNode> list = new ArrayList<>();
        double servings = (VELOCITY*INTERVAL_TIME)/deltal;  //每时段轨迹划分的虚拟段数
        for(TrackSegment segment:trackPeriod){
            for(int i=0;i<=servings;i++){
                VituralNode node = new VituralNode(segment.id,0,0);
                node.x_coordinate = segment.start_x+i*(segment.end_x-segment.start_x)/servings;
                node.y_coordinate = segment.start_y+i*(segment.end_y-segment.start_y)/servings;
                list.add(node);
            }
        }
        return list;

    }

    public static void blCalOptionalOrientation(CandidatePosition p,List<VituralNode> currentVirtualNodeList){
        p.vituralNodeInRange.clear();
        p.orientation_VituralNodes.clear();
        p.angle_vituralNode.clear();
        List<Double> list = new ArrayList<>();//所有虚拟节点的角度
        for(VituralNode n:currentVirtualNodeList){
            double dis = Util.distance(p.x_coordinate,p.y_coordinate,n.x_coordinate,n.y_coordinate);
            if(dis<=D){
                p.vituralNodeInRange.add(n);
                list.add(Util.angle(p.x_coordinate,p.y_coordinate,n.x_coordinate,n.y_coordinate));
                p.angle_vituralNode.put(Util.angle(p.x_coordinate,p.y_coordinate,n.x_coordinate,n.y_coordinate),n);
            }
        }
        Collections.sort(list);
        double[] predeterminedAngle = {2*PI-A/2,PI/2-A/2,PI-A/2,3*PI/2-A/2};
        for (int i=0;i<4;i++){
            double start_angle = predeterminedAngle[i];
            double end_angle = start_angle+A ;
            List<VituralNode> nodeInRange = new ArrayList<>();
            if(end_angle<2*PI){
                for(int j=0;j<list.size();j++){
                    if (list.get(j)<=end_angle&&list.get(j)>=start_angle)
                        nodeInRange.add(p.angle_vituralNode.get(list.get(j)));
                }
            }else{
                end_angle = end_angle-2*PI;
                for(int j=0;j<list.size();j++){
                    if(list.get(j)<=end_angle||list.get(j)>=start_angle)
                        nodeInRange.add(p.angle_vituralNode.get(list.get(j)));
                }
            }
            p.orientation_VituralNodes.put(start_angle,nodeInRange);
        }
    }

    public static void calOptionalOrientation(CandidatePosition p,List<VituralNode> currentVirtualNodeList){
        p.vituralNodeInRange.clear();
        p.orientation_VituralNodes.clear();
        p.angle_vituralNode.clear();
        List<Double> list = new ArrayList<>();
        for(VituralNode n:currentVirtualNodeList){
            double dis = Util.distance(p.x_coordinate,p.y_coordinate,n.x_coordinate,n.y_coordinate);
            if(dis<=D){
                p.vituralNodeInRange.add(n);
                list.add(Util.angle(p.x_coordinate,p.y_coordinate,n.x_coordinate,n.y_coordinate));
                p.angle_vituralNode.put(Util.angle(p.x_coordinate,p.y_coordinate,n.x_coordinate,n.y_coordinate),n);
            }
        }
        Collections.sort(list);
        for(int i=0;i<list.size();i++){
            double start_angle = list.get(i);
            double end_angle = start_angle+A ;
            List<VituralNode> nodeInRange = new ArrayList<>();
            if(end_angle<2*PI){
                for(int j=i;j<list.size();j++){
                    if (list.get(j)<=end_angle)
                        nodeInRange.add(p.angle_vituralNode.get(list.get(j)));
                }
            }else{
                end_angle = end_angle-2*PI;
                for (int j=i;j<list.size();j++)
                    nodeInRange.add(p.angle_vituralNode.get(list.get(j)));
                for(int j=0;j<list.size();j++){
                    if(list.get(j)<=end_angle)
                        nodeInRange.add(p.angle_vituralNode.get(list.get(j)));
                }
            }
            p.orientation_VituralNodes.put(list.get(i),nodeInRange);
        }
    }

    public static void contrastAlgorithmChooseOrientation(CandidatePosition p){
        p.trackSegmentsInRange.clear();

        double bestOrientation=0;
        double max_utility = 0;
        for (Map.Entry<Double,List<VituralNode>> entry:p.orientation_VituralNodes.entrySet()){
            Set<Integer> set = new HashSet<>();
            double startOrientation = entry.getKey();
            double endOrientation = startOrientation+A;
            for(VituralNode n:entry.getValue()){
                set.add(n.id); //统计该方向内的轨迹
            }
            double utility =0;
            for (Integer j:set){//遍历该朝向内的轨迹，计算效用 第几个传感器
                for (int k=0;k<sensorsTrack.get(j).size();k++){
                    TrackSegment segment = sensorsTrack.get(j).get(k);
                    //分割小段找每段中点
                    Map<Double,VituralNode> midPoint = new TreeMap<>();
                    List<VituralNode> endPoint = new ArrayList<>();//每小段的端点
                    double servings = (VELOCITY*INTERVAL_TIME)/deltal;  //每时段轨迹划分的虚拟段数
                    for(int i=0;i<=servings;i++){
                        VituralNode node = new VituralNode(segment.id,0,0);
                        node.x_coordinate = segment.start_x+i*(segment.end_x-segment.start_x)/servings;
                        node.y_coordinate = segment.start_y+i*(segment.end_y-segment.start_y)/servings;
                        endPoint.add(node);
                    }
                    for(int i=1;i<endPoint.size();i++){
                        VituralNode node = new VituralNode(segment.id,0,0);
                        node.x_coordinate = (endPoint.get(i).x_coordinate+endPoint.get(i-1).x_coordinate)/2;
                        node.y_coordinate = (endPoint.get(i).y_coordinate+endPoint.get(i-1).y_coordinate)/2;
                        double angle = Util.angle(p.x_coordinate,p.y_coordinate,node.x_coordinate,node.y_coordinate);
                        midPoint.put(angle,node);
                    }

                    for(Map.Entry<Double,VituralNode> entry1:midPoint.entrySet()){
                        double angle = entry1.getKey();
                        if (endOrientation<2*PI){
                            if(angle>=startOrientation&&angle<=endOrientation){
                                utility += Util.calutility(p.x_coordinate,p.y_coordinate,entry1.getValue().x_coordinate,entry1.getValue().y_coordinate);
                            }

                        }else{
                            endOrientation -= 2*PI;
                            if (angle>=startOrientation||angle<=endOrientation){
                                utility += Util.calutility(p.x_coordinate,p.y_coordinate,entry1.getValue().x_coordinate,entry1.getValue().y_coordinate);
                            }

                        }
                    }
                }
            }
            if (utility>max_utility){
                max_utility = utility;
                bestOrientation = startOrientation;
            }
        }
        p.start_orientation = bestOrientation;
        p.utility = max_utility;
    }


    public static void calBestOrientation(CandidatePosition p,int interval){//阶段为interval
        p.trackSegmentsInRange.clear();

        double bestOrientation=0;
        double max_utility = 0;
        for (Map.Entry<Double,List<VituralNode>> entry:p.orientation_VituralNodes.entrySet()){
            Set<Integer> set = new HashSet<>();
            double startOrientation = entry.getKey();
            double endOrientation = startOrientation+A;
            for(VituralNode n:entry.getValue()){
                set.add(n.id); //统计该方向内的轨迹
            }
            double utility =0;
            for (Integer j:set){//遍历该朝向内的轨迹，计算效用
                TrackSegment segment = sensorsTrack.get(j).get(interval-1);
                //分割小段找每段中点
                Map<Double,VituralNode> midPoint = new TreeMap<>();
                List<VituralNode> endPoint = new ArrayList<>();//每小段的端点
                double servings = (VELOCITY*INTERVAL_TIME)/deltal;  //每时段轨迹划分的虚拟段数
                for(int i=0;i<=servings;i++){
                    VituralNode node = new VituralNode(segment.id,0,0);
                    node.x_coordinate = segment.start_x+i*(segment.end_x-segment.start_x)/servings;
                    node.y_coordinate = segment.start_y+i*(segment.end_y-segment.start_y)/servings;
                    endPoint.add(node);
                }
                for(int i=1;i<endPoint.size();i++){
                    VituralNode node = new VituralNode(segment.id,0,0);
                    node.x_coordinate = (endPoint.get(i).x_coordinate+endPoint.get(i-1).x_coordinate)/2;
                    node.y_coordinate = (endPoint.get(i).y_coordinate+endPoint.get(i-1).y_coordinate)/2;
                    double angle = Util.angle(p.x_coordinate,p.y_coordinate,node.x_coordinate,node.y_coordinate);
                    midPoint.put(angle,node);
                }

                for(Map.Entry<Double,VituralNode> entry1:midPoint.entrySet()){
                    double angle = entry1.getKey();
                    if (endOrientation<2*PI){
                        if(angle>=startOrientation&&angle<=endOrientation){
                            utility += Util.calutility(p.x_coordinate,p.y_coordinate,entry1.getValue().x_coordinate,entry1.getValue().y_coordinate);
                        }

                    }else{
                        endOrientation -= 2*PI;
                        if (angle>=startOrientation||angle<=endOrientation){
                            utility += Util.calutility(p.x_coordinate,p.y_coordinate,entry1.getValue().x_coordinate,entry1.getValue().y_coordinate);
                        }

                    }
                }
            }
            if (utility>max_utility){
                max_utility = utility;
                bestOrientation = startOrientation;
            }
        }
        p.start_orientation = bestOrientation;
        p.utility = max_utility;
    }
}
