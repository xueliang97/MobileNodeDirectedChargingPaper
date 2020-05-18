package com.hdu.paper2code;

import java.util.HashMap;
import java.util.Map;

public class test {
    public static void main(String[] args){
        Map<String,String> map = new HashMap<String,String>();
        map.put("熊大", "棕色");
        map.put("熊二", "黄色");
        for(Map.Entry<String, String> entry : map.entrySet()){
            String mapKey = entry.getKey();
            String mapValue = entry.getValue();
            System.out.println(mapKey+":"+mapValue);
        }
    }
}
