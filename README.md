# avoid_side_Aria
@author：zhw    
@time:2016-3-17    
@content:    
    主要修改文件 /include/ArActionAvoidSide.h  /src/ArActionAvoidSide.cpp  /examples/avoid_obstacles_zhw.cpp    
    其中/include/ArActionAvoidSide.h 对应一些变量定义    
       /src/ArActionAvoidSide.cpp 里面基于 Araction 开发，可以类比于周期为100ms 的定时器中断，在其中做避障和导航算法    
       /examples/avoid_obstacles_zhw.cpp 为main 函数，做上层控制和调度    
    使用方法：    
        编译及运行：    
            $ cd /usr/local/Aria    
            $ make    
            $ make install    
            $ cd /usr/local/Aria/examples    
            $ make
            $ ./avoid_obstacles_zhw -rp /dev/ttyUSB0
        
        
