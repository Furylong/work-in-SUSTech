%建立机械臂模型
d = [0 0 0 0 0];
a = [0 1000 1000 1000 0];
alpha = [pi/2 0 0 0 pi/2];

L(1) = Link([0 d(1) a(1) alpha(1)],'modified'); 
L(1).qlim = [-150 150]*pi/180;
L(2) = Link([0 d(2) a(2) alpha(2)],'modified'); 
L(2).qlim = [-150 150]*pi/180;
L(2).offset = pi/2;
L(3) = Link([0 d(3) a(3) alpha(3)],'modified'); 
L(3).qlim = [-150 150]*pi/180;
L(4) = Link([0 d(4) a(4) alpha(4)],'modified'); 
L(4).qlim = [-150 150]*pi/180;
L(4).offset = pi/2;
L(5) = Link([0 d(5) a(5) alpha(5)],'modified'); 
L(5).qlim = [-150 150]*pi/180;

robot = SerialLink(L,'name','robot');
%定义基坐标和工具坐标
MyRobot.base = transl(0,0,0);
MyRobot.tool = transl(0,0,100);

robot.plot([0 0 0 0 0]);