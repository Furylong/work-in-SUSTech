%导入模型
%mdl_5dof;
%采用关节空间轨迹规划
%给出关节角度
%前六个为手臂移动后读到的值
syms x1 x2 x3 x4 x5 x6;
syms y1 y2 y3 y4 y5 y6;
syms z1 z2 z3 z4 z5 z6;
syms a1 a2 a3 a4 a5 a6;

mdl_Dyn_5dof;


    lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
  [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h','addheader', 'group_sync_write.h');
end

% Control table address
ADDR_MX_TORQUE_ENABLE       = 24;           % Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30;
ADDR_MX_PRESENT_POSITION    = 36;
 ADDR_MX_MOVING_SPEED       = 32;

%Data Byte Length
LEN_MX_GOAL_POSITION = 2;
LEN_MX_PRESENT_POSITION = 2;

% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL1_ID = 9;            % Dynamixel#1 ID: 1
DXL2_ID = 1;            % Dynamixel#2 ID: 2
DXL3_ID = 3;            % Dynamixe1#3 ID: 3
DXL4_ID = 2;            % Dynamixel#2 ID: 4
DXL5_ID = 7;            % Dynamixe1#3 ID: 5 
DXL6_ID = 8;            % Dynamixel#2 ID: 6
DXL7_ID = 4;            % Dynamixel#2 ID: 7
DXL8_ID = 5;            % Dynamixel#2 ID: 8
DXL9_ID = 6;            % Dynamixel#2 ID: 9
BAUDRATE                    = 1000000;
DEVICENAME1                 = 'COM7';       % Check which port is being used on your controller
DEVICENAME2                 = 'COM7';       % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
DEVICENAME3                 = 'COM7'; 
DEVICENAME4                 = 'COM7'; 
DEVICENAME5                 = 'COM7'; 
DEVICENAME6                 = 'COM7'; 

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 500;          % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 700;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num  = portHandler(DEVICENAME1);
port_num1 = portHandler(DEVICENAME1);
port_num2 = portHandler(DEVICENAME2);
port_num3 = portHandler(DEVICENAME3);
port_num4 = portHandler(DEVICENAME4);
port_num5 = portHandler(DEVICENAME5);
port_num6 = portHandler(DEVICENAME6);

% Initialize PacketHandler Structs
packetHandler();

% Initialize Groupsyncwrite instance
group_num1 = groupSyncWrite(port_num2, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
group_num2 = groupSyncWrite(port_num3, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
group_num3 = groupSyncWrite(port_num4, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

%open port
port = [port_num1 port_num2 port_num3 port_num4 port_num5 port_num6];
for i = 1:6
    if(openPort(port(i)))
         fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
    end
end

%Set port baudrate
for i = 1:6
    if (setBaudRate(port(i), BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
    else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
    end
end

index = 1;
dxl_comm_result = COMM_TX_FAIL;             % Communication result
dxl_addparam_result = false;                % AddParam result

dxl_error = 0;                              % Dynamixel error
dxl1_present_position = 0;                  % Present position
dxl2_present_position = 0;
dxl3_present_position = 0;
dxl4_present_position = 0;
dxl5_present_position = 0;
dxl6_present_position = 0;
dxl7_present_position = 0;
dxl8_present_position = 0;
dxl9_present_position = 0;

% Enable Dynamixel#1 Torque
write1ByteTxRx(port_num1, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num1, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num1, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num1, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num1, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel#2 Torque
write1ByteTxRx(port_num2, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num2, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num2, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num2, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num2, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel#3 Torque
write1ByteTxRx(port_num3, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num3, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num3, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num3, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num3, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel#4 Torque
write1ByteTxRx(port_num4, PROTOCOL_VERSION, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num4, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num4, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num4, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num4, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel#5 Torque
write1ByteTxRx(port_num5, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num5, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num5, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num5, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num5, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel#6 Torque
write1ByteTxRx(port_num6, PROTOCOL_VERSION, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num6, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num6, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num6, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num6, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel#7 Torque
write1ByteTxRx(port_num2, PROTOCOL_VERSION, DXL7_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num2, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num2, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num2, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num2, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel#8 Torque
write1ByteTxRx(port_num3, PROTOCOL_VERSION, DXL8_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num3, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num3, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num3, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num3, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel#9 Torque
write1ByteTxRx(port_num4, PROTOCOL_VERSION, DXL9_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num4, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num4, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num4, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num4, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

T1 = robot.fkine([0 0 0 0 0]*pi/180);
count = 0;
h = 0;
    b1 = 0;
    b2 = 0;
    b3 = 0;
    b4 = 0;
    b5 = 0;
    b6 = 0;
    a1 = 0;
    a2 = 0;
    a3 = 0;
    a4 = 0;
    a5 = 0;
while 1
    
    
    t = 0:0.001:0.001;
    tem = pwd;
    cd('C:\Users\11634\Desktop\机建控\project代码\output10\output');
    filename1 = "output_" + count + ".csv";
    filename2 = "output_" + count + "_of_joint_12.csv";
    filename3 = "output_" + count + "_of_joint_14.csv";
    filename4 = "output_" + count + "_of_joint_16.csv";
    filename5 = "output_" + count + "_of_joint_18.csv";
    filename6 = "output_" + count + "_of_joint_22.csv";
    filename7 = "output_" + count + "_of_joint_24.csv";
    
    c = 0.3;
    temp1 = csvread(filename1);
    q = temp1(1)*pi/180*c; w = temp1(2)*pi/180*c; t = temp1(3)*pi/180*c;

    temp2 = csvread(filename2);%12
    x2 = temp2(2); y2 = temp2(3); z2 = temp2(4);
    temp3 = csvread(filename3);%14
    x3 = temp3(2); y3 = temp3(3); z3 = temp3(4);
    temp4 = csvread(filename4);%16
    x4 = temp4(2); y4 = temp4(3); z4 = temp4(4);
    temp5 = csvread(filename5);%18
    x5 = temp5(2); y5 = temp5(3); z5 = temp5(4);    
    temp1 = csvread(filename6);%22
    x6 = temp1(2); y6 = temp1(3); z6 = temp1(4);
    temp6 = csvread(filename7);%24
    x1 = temp6(2); y1 = temp6(3); z1 = temp6(4);
    cd (tem);
    
   
    
     if h==0
        x11 = x1;x21 = x2;x31 = x3;x41 = x4;x51 = x5;x61 = x6;
        y11 = y1;y21 = y2;y31 = y3;y41 = y4;y51 = y5;y61 = y6;
        z11 = z1;z21 = z2;z31 = z3;z41 = z4;z51 = z5;z61 = z6;
        h = h+1;
        count = count +1;
        continue;
     end
     
     a1 = (atan2(-(z5-z51),-(x5-x51)))*c;
     a5 = (atan2(-(z6-z61),-(y6-y61)))*c;
     
     a2 =  c* (acos(([x3-x2 y3-y2 z3-z2]*[x1-x2 y1-y2 z1-z2]')/(norm([x3-x2 y3-y2 z3-z2])*norm([x1-x2 y1-y2 z1-z2])))-acos(([x31-x21 y31-y21 z31-z21]*[x11-x21 y11-y21 z11-z21]')/(norm([x31-x21 y31-y21 z31-z21])*norm([x11-x21 y11-y21 z11-z21]))));
     a3 =  c* (acos(([x4-x3 y4-y3 z4-z3]*[x2-x3 y2-y3 z2-z3]')/(norm([x4-x3 y4-y3 z4-z3])*norm([x2-x3 y2-y3 z2-z3])))-acos(([x41-x31 y41-y31 z41-z31]*[x21-x31 y21-y31 z21-z31]')/(norm([x41-x31 y41-y31 z41-z31])*norm([x21-x31 y21-y31 z21-z31]))));
     a4 =  c* (acos(([x5-x4 y5-y4 z5-z4]*[x3-x4 y3-y4 z3-z4]')/(norm([x5-x4 y5-y4 z5-z4])*norm([x3-x4 y3-y4 z3-z4])))-acos(([x51-x41 y51-y41 z51-z41]*[x31-x41 y31-y41 z31-z41]')/(norm([x51-x41 y51-y41 z51-z41])*norm([x31-x41 y31-y41 z31-z41]))));
     
%     a1 = (atan2(-(z5-z51),-(x5-x51)))*c;
%     a2 = c* acos(([x31-x21 y31-y21 z31-z21]*[x3-x2 y3-y2 z3-z2]')/(norm([x31-x21 y31-y21 z31-z21])*norm([x3-x2 y3-y2 z3-z2])));% 夹角°
%     a3 = c* acos(([x41-x31 y41-y31 z41-z31]*[x4-x3 y4-y3 z4-z3]')/(norm([x41-x31 y41-y31 z41-z31])*norm([x4-x3 y4-y3 z4-z3])));
%     a4 = c* acos(([x51-x41 y51-y41 z51-z41]*[x5-x4 y5-y4 z5-z4]')/(norm([x51-x41 y51-y41 z51-z41])*norm([x5-x4 y5-y4 z5-z4])));
%     %a2 = atan2((x3-x2),(y3-y2));
%     %a3 = atan2((x4-x3),(y4-y3));
%     a5 = (atan2(-(z6-z61),-(y6-y61)))*c;
%     %a5 = atan2((y6-y5-dy),(x6-x5-dx));
%    
     x11 = x1;x21 = x2;x31 = x3;x41 = x4;x51 = x5;x61 = x6;
     y11 = y1;y21 = y2;y31 = y3;y41 = y4;y51 = y5;y61 = y6;
     z11 = z1;z21 = z2;z31 = z3;z41 = z4;z51 = z5;z61 = z6;
     %正运动学
     A1 = [cos(a1) 0 -sin(a1) 0;sin(a1) 0 cos(a1) 0;0 -1 0 l1;0 0 0 1];
     A2 = [cos(a2) -sin(a2) 0 l2*cos(a2);sin(a2) cos(a2) 0 l2*sin(a2);0 0 1 0;0 0 0 1];
     A3 = [cos(a3) -sin(a3) 0 l3*cos(a3);sin(a3) cos(a3) 0 l3*sin(a3);0 0 1 0;0 0 0 1];
     A4 = [cos(a4) 0 -sin(a4) 0;sin(a4) 0 cos(a4) 0;0 -1 0 0;0 0 0 1];
     A5 = [cos(a5) -sin(a5) 0 0;sin(a5) cos(a5) 0 0;0 0 1 l5;0 0 0 1];
     T2 = A1*A2*A3*A4*A5;
     
     %逆运动学
     x5x = cos(a1)*cos(a2+a3+a4)*cos(a5)+sin(a1)*sin(a5);
     x5y = sin(a1)*cos(a2+a3+a4)*cos(a5)+cos(a1)*sin(a5);
     x5z = -sin(a2+a3+a4)*cos(a5);
     y5x = -cos(a1)*cos(a2+a3+a4)*sin(a5)+sin(a1)*cos(a5);
     y5y = -sin(a1)*cos(a2+a3+a4)*sin(a5)-sin(a1)*cos(a5);
     y5z = sin(a2+a3+a4)*sin(a5);
     z5x = -cos(a1)*sin(a2+a3+a4);
     z5y = -sin(a1)*sin(a2+a3+a4);
     z5z = cos(a2+a3+a4);
     p5x = cos(a1)*(-l5*sin(a2+a3+a4)+l3*cos(a2+a3)+l2*cos(a2));
     p5y = sin(a1)*(-l5*sin(a2+a3+a4)+l3*cos(a2+a3)+l2*cos(a2));
     p5z = l1-l2*sin(a2)-l3*cos(a2+a3)-l5*cos(a2+a3+a4);
     
     i = l1-l5*cos(a2+a3+a4)-p5z;
     j = p5x*cos(a1)+p5y*sin(a1)+l5*sin(a2+a3+a4);
     q1 = real(atan2(p5y,p5x));
     q2 = real(atan2(i*(l2+l3*cos(a3))-j*l3*sin(a3),i*l3*sin(a3)+j*(l2+l3*cos(a3))));
     q3 = real(acos((i^2+j^2-l2^2-l3^2)/2/l2/l3));
     q4 = real(0-q2-q3);
     q5 = real(cos(a2+a3+a4)*q1-2*atan2(x5y,x5x));
     qqq = [q1 q4 q5];
     for i = 1:3
         if (qqq(i)> pi/2)
             qqq(i) = pi/2;
         end
         if (qqq(i)< (-1*pi/2))
             qqq(i) = -pi/2;
         end
     end
    if q2<0
        q2 = 0;
    end
    if q2>2/3*pi
        q2 = 2/3*pi;
    end
    if q3<0
        q3 = 0;
    end
    if q3<(-1*2/3*pi)
        q3 = -2/3*pi;
    end
%     if(a1<-45*pi/180)
%         a1 = -45*pi/180;
%     end
%     if(a1>65*pi/180)
%         a1 = 65*pi/180;
%     end
%     if(a2<-45*pi/180)
%         a2 = -45*pi/180;
%     end
%     if(a2>65*pi/180)
%         a2 = 65*pi/180;
%     end
%     if(a3<-45*pi/180)
%         a3 = -45*pi/180;
%     end
%     if(a3>65*pi/180)
%         a3 = 65*pi/180;
%     end
%     if(a4<-45*pi/180)
%         a4 = -45*pi/180;
%     end
%     if(a4>65*pi/180)
%         a4 = 65*pi/180;
%     end
%     if(a5<-45*pi/180)
%         a5 = -45*pi/180;
%     end
%     if(a5>65*pi/180)
%         a5 = 65*pi/180;
%     end
    
    
    count = count + 8;
%     T2 = robot.fkine([a1 a2 a3 a4 a5]);
    
%     q1 = robot.ikine(T1,'mask',[1 1 1 1 1 0]);
%     q2 = robot.ikine(T2,'mask',[1 1 1 1 1 0],'q0',q1);
% 
%     [q,qt2,qtt] = jtraj(q1,q2,t);
%     qt = qt2(2,:);
%     for i=1:5
%         qt(i) = abs(qt(i));
%     end
%     t = MDH_Dyn(robot,q,qt,qtt);
%     aa = [a1,a2,a4,a5,a3];
%     for i = 1:4
%         if (aa(i)<-130*pi/180)
%             aa(i) = -125*180/pi;
%         end
%         if (aa(i)>130*pi/180)
%             aa(i) = 125*180/pi;
%         end
%     end
%     
%     if(aa(5)<-70*pi/180)
%         aa(5) = -65*180/pi;
%     end
%     if(aa(5)>150*pi/180)
%         aa(5) = 145*180/pi;
%     end;;'

   %robot.plot(q);

   
    %fprintf(t);
%     T1 = T2;
    b =1;
    d = 1;
%while 1
%     if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
%         break;
%     end

%while 1
%     if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
%         break;
%     end
      angle1 = q1*180/pi;                                         % Goal position
      angle2 = q2*180/pi;                                         %group1
      angle3 = -q3*180/pi                                        %group2     
      angle4 = -q4*180/pi;                                         %group3
      angle5 = q5*180/pi;
      angle6 = 0;
     q3
     
%       angle1 = b1+angle1;%-150 150
%       angle2 = b2+angle2;%-120 0
%       angle3 = b3+angle3;% -120 0
%       angle4 = b4+angle4;%-150 150
%       angle5 = b5+angle5;%-150 151
%       angle6 = b6+angle6;
      if (angle1<-150)
          angle1 = -150;
      end
      if (angle1>150)
          angle1 = 150;
      end
       if (angle2<-120)
          angle2 = -120;
      end
      if (angle2>0)
          angle2 = 0;
      end
%        if (angle3<-180)
%           angle3 = -180;
%       end
%       if (angle3>40)
%           angle3 = 40;
%       end
       if (angle4<-150)
          angle4 = -150;
      end
      if (angle4>150)
          angle4 = 150;
      end
       if (angle5<-150)
          angle5 = -150;
      end
      if (angle5>150)
          angle5 = 150;
      end
      
    

%       
%       b1 = angle1
%       b2 = angle2
%       b3 = angle3
%       b4 = angle4
%       b5 = angle5
%       b6 = angle6
%     angle1 = b*a1*180/pi;                                         % Goal position
%     angle2 = b*a2*180/pi;                                         %group1
%     angle3 = b*a3*180/pi;                                         %group2     
%     angle4 = b*a4*180/pi;                                         %group3
%     angle5 = b*a5*180/pi;
%     angle6 = 0;
% 
%     angle1 = 10;                                         % Goal position
%     angle2 = 10;                                         %group1
%     angle3 = -60;                                         %group2     
%     angle4 = 10;                                         %group3
%     angle5 = 10;
%     angle6 = 0;

    dxl_goal_position12 = d*(1023-(round(angle2*1023/300)+810));         
    dxl_goal_position11 = d*(round(angle2*1023/300)+810);
    dxl_goal_position21 = d*(round(angle3*1023/300)+810);         
    dxl_goal_position22 = d*(1023-(round(angle3*1023/300)+810));
    dxl_goal_position32 = d*(round(angle4*1023/300)+512);
    dxl_goal_position31 = d*(1023-(round(angle4*1023/300)+511));
    dxl_goal_position1 = d*(round(angle1*1023/300)+512);
    dxl_goal_position5 = d*(round(angle5*1023/300)+512);
    dxl_goal_position6 = d*(800);
%     omega1 = qt(1);                                           % Goal angular velocity(rad/s)
%     omega2 = qt(2);                                           %group1
%     omega3 = qt(3);                                           %group2
%     omega4 = qt(4);                                           %group3
%     omega5 = qt(5);
%     omega6 = 1023;
    dxl_goal_angular_velocity1 = (400);
    dxl_goal_angular_velocity2 = (400);
    dxl_goal_angular_velocity3 = (400);
    dxl_goal_angular_velocity4 = (400);
    dxl_goal_angular_velocity5 = (400);
    dxl_goal_angular_velocity6 = (400);
      
    % The first group
    % Add Dynamixel#2 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num1, DXL2_ID,  dxl_goal_position11, LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL2_ID);
        return;
    end

    % Add Dynamixel#7 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num1, DXL7_ID,  dxl_goal_position12, LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL7_ID);
        return;
    end
    
    % The second group
    % Add Dynamixel#3 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num2, DXL3_ID, dxl_goal_position21, LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL3_ID);
        return;
    end

    % Add Dynamixel#8 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num2, DXL8_ID, dxl_goal_position22, LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL8_ID);
        return;
    end
    
    % The third group
    % Add Dynamixel#4 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num3, DXL4_ID, dxl_goal_position31, LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL4_ID);
        return;
    end

    % Add Dynamixel#9 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num3, DXL9_ID, dxl_goal_position32, LEN_MX_GOAL_POSITION);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL9_ID);
        return;
    end
    
    % Syncwrite goal position
    groupSyncWriteTxPacket(group_num1);
    if getLastTxRxResult(port_num2, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num2, PROTOCOL_VERSION));
    end
    
    groupSyncWriteTxPacket(group_num2);
    if getLastTxRxResult(port_num3, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num3, PROTOCOL_VERSION));
    end
    
    groupSyncWriteTxPacket(group_num3);
    if getLastTxRxResult(port_num4, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num4, PROTOCOL_VERSION));
    end
    
    % Clear syncwrite parameter storage
    groupSyncWriteClearParam(group_num1);
    groupSyncWriteClearParam(group_num2);
    groupSyncWriteClearParam(group_num3);
    
    % Write Dynamixel#1 goal position
    write2ByteTxRx(port_num1, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position1);
    if getLastTxRxResult(port_num1, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num1, PROTOCOL_VERSION));
    elseif getLastRxPacketError(port_num1, PROTOCOL_VERSION) ~= 0
        printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num1, PROTOCOL_VERSION));
    end
    
     % Write Dynamixel#5 goal position
    write2ByteTxRx(port_num5, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position5);
    if getLastTxRxResult(port_num5, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num5, PROTOCOL_VERSION));
    elseif getLastRxPacketError(port_num5, PROTOCOL_VERSION) ~= 0
        printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num5, PROTOCOL_VERSION));
    end
    
     % Write Dynamixel#6 goal position
    write2ByteTxRx(port_num6, PROTOCOL_VERSION, DXL6_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position6);
    if getLastTxRxResult(port_num6, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num6, PROTOCOL_VERSION));
    elseif getLastRxPacketError(port_num6, PROTOCOL_VERSION) ~= 0
        printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num6, PROTOCOL_VERSION));
    end
    
      %Write Dynamixel#1 moving speed
    write2ByteTxRx(port_num1, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_MOVING_SPEED, dxl_goal_angular_velocity1);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
     %Write Dynamixel#2 moving speed
    write2ByteTxRx(port_num2, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_MOVING_SPEED, dxl_goal_angular_velocity2);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
     %Write Dynamixel#7 moving speed
     write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL7_ID, ADDR_MX_MOVING_SPEED, dxl_goal_angular_velocity2);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
     %Write Dynamixel#3 moving speed
    write2ByteTxRx(port_num3, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_MOVING_SPEED,  dxl_goal_angular_velocity3);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
     %Write Dynamixel#8 moving speed
     write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL8_ID, ADDR_MX_MOVING_SPEED,  dxl_goal_angular_velocity3);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
     %Write Dynamixel#4 moving speed
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_MX_MOVING_SPEED, dxl_goal_angular_velocity4);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
     %Write Dynamixel#9 moving speed
     write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL9_ID, ADDR_MX_MOVING_SPEED, dxl_goal_angular_velocity4);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
     %Write Dynamixel#5 moving speed
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_MOVING_SPEED, dxl_goal_angular_velocity5);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
     %Write Dynamixel#6 moving speed
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL6_ID, ADDR_MX_MOVING_SPEED, dxl_goal_angular_velocity6);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

%     %while 1
%       % Read Dynamixel#1 present position
%       dxl1_present_position = read2ByteTxRx(port_num1, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_PRESENT_POSITION);
%       dxl_comm_result = getLastTxRxResult(port_num1, PROTOCOL_VERSION);
%       dxl_error = getLastRxPacketError(port_num1, PROTOCOL_VERSION);
%       if dxl_comm_result ~= COMM_SUCCESS
%           fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%       elseif dxl_error ~= 0
%           fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%       end
% 
%       % Read Dynamixel#2 and 7 present position
%       dxl2_present_position = read2ByteTxRx(port_num2, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_PRESENT_POSITION);
%       if getLastTxRxResult(port_num2, PROTOCOL_VERSION) ~= COMM_SUCCESS
%             printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num2, PROTOCOL_VERSION));
%       elseif getLastRxPacketError(port_num2, PROTOCOL_VERSION) ~= 0
%             printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num2, PROTOCOL_VERSION));
%       end
% 
%       dxl7_present_position = read2ByteTxRx(port_num2, PROTOCOL_VERSION, DXL7_ID, ADDR_MX_PRESENT_POSITION);
%       if getLastTxRxResult(port_num2, PROTOCOL_VERSION) ~= COMM_SUCCESS
%             printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num2, PROTOCOL_VERSION));
%       elseif getLastRxPacketError(port_num2, PROTOCOL_VERSION) ~= 0
%             printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num2, PROTOCOL_VERSION));
%       end
%       
%       % Read Dynamixel#3 and 8 present position
%       dxl3_present_position = read2ByteTxRx(port_num3, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_PRESENT_POSITION);
%       if getLastTxRxResult(port_num3, PROTOCOL_VERSION) ~= COMM_SUCCESS
%             printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num3, PROTOCOL_VERSION));
%       elseif getLastRxPacketError(port_num3, PROTOCOL_VERSION) ~= 0
%             printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num3, PROTOCOL_VERSION));
%       end
% 
%       dxl8_present_position = read2ByteTxRx(port_num3, PROTOCOL_VERSION, DXL8_ID, ADDR_MX_PRESENT_POSITION);
%       if getLastTxRxResult(port_num3, PROTOCOL_VERSION) ~= COMM_SUCCESS
%             printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num3, PROTOCOL_VERSION));
%       elseif getLastRxPacketError(port_num3, PROTOCOL_VERSION) ~= 0
%             printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num3, PROTOCOL_VERSION));
%       end
%       
%       % Read Dynamixel#4 and 9 present position
%       dxl4_present_position = read2ByteTxRx(port_num4, PROTOCOL_VERSION, DXL4_ID, ADDR_MX_PRESENT_POSITION);
%       if getLastTxRxResult(port_num4, PROTOCOL_VERSION) ~= COMM_SUCCESS
%             printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num4, PROTOCOL_VERSION));
%       elseif getLastRxPacketError(port_num4, PROTOCOL_VERSION) ~= 0
%             printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num4, PROTOCOL_VERSION));
%       end
% 
%       dxl9_present_position = read2ByteTxRx(port_num4, PROTOCOL_VERSION, DXL9_ID, ADDR_MX_PRESENT_POSITION);
%       if getLastTxRxResult(port_num4, PROTOCOL_VERSION) ~= COMM_SUCCESS
%             printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num4, PROTOCOL_VERSION));
%       elseif getLastRxPacketError(port_num4, PROTOCOL_VERSION) ~= 0
%             printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num4, PROTOCOL_VERSION));
%       end
%       
%       % Read Dynamixel#5 present position
%       dxl5_present_position = read2ByteTxRx(port_num5, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_PRESENT_POSITION);
%       dxl_comm_result = getLastTxRxResult(port_num5, PROTOCOL_VERSION);
%       dxl_error = getLastRxPacketError(port_num5, PROTOCOL_VERSION);
%       if dxl_comm_result ~= COMM_SUCCESS
%           fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%       elseif dxl_error ~= 0
%           fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%       end
%       
%       % Read Dynamixel#6 present position
%       dxl6_present_position = read2ByteTxRx(port_num6, PROTOCOL_VERSION, DXL6_ID, ADDR_MX_PRESENT_POSITION);
%       dxl_comm_result = getLastTxRxResult(port_num6, PROTOCOL_VERSION);
%       dxl_error = getLastRxPacketError(port_num6, PROTOCOL_VERSION);
%       if dxl_comm_result ~= COMM_SUCCESS
%           fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%       elseif dxl_error ~= 0
%           fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%       end
      
%         fprintf('1') ;
       fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n', DXL1_ID, dxl_goal_position1, dxl1_present_position, DXL2_ID, dxl_goal_position11, dxl2_present_position, DXL3_ID, dxl_goal_position21, dxl3_present_position, DXL4_ID, dxl_goal_position31, dxl4_present_position, DXL5_ID, dxl_goal_position5, dxl5_present_position, DXL6_ID, dxl_goal_position6, dxl6_present_position);

%       if ~((abs(dxl_goal_position1 - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position11 - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)||(abs(dxl_goal_position21 - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD) ||(abs(dxl_goal_position31 - dxl4_present_position) > DXL_MOVING_STATUS_THRESHOLD) ||(abs(dxl_goal_position5 - dxl5_present_position) > DXL_MOVING_STATUS_THRESHOLD) ||(abs(dxl_goal_position6 - dxl6_present_position) > DXL_MOVING_STATUS_THRESHOLD))
%           break;
%       end
    %end
%end
end

% Disable Dynamixel#1 Torque
write1ByteTxRx(port_num1, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num1, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num1, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num1, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num1, PROTOCOL_VERSION));
end

% Disable Dynamixel#2 Torque
write1ByteTxRx(port_num2, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num2, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num2, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num2, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num2, PROTOCOL_VERSION));
end

% Disable Dynamixel#3 Torque
write1ByteTxRx(port_num3, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num3, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num3, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num3, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num3, PROTOCOL_VERSION));
end

% Disable Dynamixel#4 Torque
write1ByteTxRx(port_num4, PROTOCOL_VERSION, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num4, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num4, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num4, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num4, PROTOCOL_VERSION));
end

% Disable Dynamixel#5 Torque
write1ByteTxRx(port_num5, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num5, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num5, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num5, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num5, PROTOCOL_VERSION));
end

% Disable Dynamixel#6 Torque
write1ByteTxRx(port_num6, PROTOCOL_VERSION, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num6, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num6, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num6, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num6, PROTOCOL_VERSION));
end

% Disable Dynamixel#7 Torque
write1ByteTxRx(port_num2, PROTOCOL_VERSION, DXL7_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num2, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num2, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num2, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num2, PROTOCOL_VERSION));
end

% Disable Dynamixel#8 Torque
write1ByteTxRx(port_num3, PROTOCOL_VERSION, DXL8_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num3, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num3, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num3, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num3, PROTOCOL_VERSION));
end

% Disable Dynamixel#9 Torque
write1ByteTxRx(port_num4, PROTOCOL_VERSION, DXL9_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num4, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num4, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num4, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num4, PROTOCOL_VERSION));
end

% % Close every port
% for i = 1:6
%     closePort(port(i));
% end


% Unload Library
unloadlibrary(lib_name);

close all;
