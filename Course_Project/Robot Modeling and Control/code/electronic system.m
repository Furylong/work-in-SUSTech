
%代入关节角度

%si = [20 50 -30 -25 -10]*pi/180;
%sf = [70 10 -60 -50 30]*pi/180;

%t = 0:0.1:4;
%T1 = robot.fkine(si);
%T2 = robot.fkine(sf);
mdl_Dyn_5dof;
count = 0;

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

while 1
   
    

    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end
    angle1 = a1*180/pi;                                         % Goal position
    angle2 = a2*180/pi;                                         %group1
    angle3 = a3*180/pi;                                         %group2     
    angle4 = a4*180/pi;                                         %group3
    angle5 = a5*180/pi;
    angle6 = 0;
    dxl_goal_position11 = [round(angle2*1023/300)+512];         
    dxl_goal_position12 = [1023-(round(angle2*1023/300)+512)];
    dxl_goal_position21 = [round(angle3*1023/300)+512];         
    dxl_goal_position22 = [1023-(round(angle3*1023/300)+512)];
    dxl_goal_position31 = [round(angle4*1023/300)+512];
    dxl_goal_position32 = [1023-(round(angle4*1023/300)+512)];
    dxl_goal_position1 = [1023];
    dxl_goal_position5 = [round(angle5*1023/300)+512];
    dxl_goal_position6 = [round(angle6*1023/300)+512];
    omega1 = qt(1);                                           % Goal angular velocity(rad/s)
    omega2 = qt(2);                                           %group1
    omega3 = qt(3);                                           %group2
    omega4 = qt(4);                                           %group3
    omega5 = qt(5);
    omega6 = 1023;
    dxl_goal_angular_velocity1 = [round(omega1/0.666)];
    dxl_goal_angular_velocity2 = [round(omega2/0.666)];
    dxl_goal_angular_velocity3 = [round(omega3/0.666)];
    dxl_goal_angular_velocity4 = [round(omega4/0.666)];
    dxl_goal_angular_velocity5 = [round(omega5/0.666)];
    dxl_goal_angular_velocity6 = [round(omega6/0.666)];
      
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
    write2ByteTxRx(port_num6, PROTOCOL_VERSION, DXL6_ID, ADDR_MX_MOVING_SPEED, dxl_goal_angular_velocity6);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    %while 1
      % Read Dynamixel#1 present position
      dxl1_present_position = read2ByteTxRx(port_num1, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_PRESENT_POSITION);
      dxl_comm_result = getLastTxRxResult(port_num1, PROTOCOL_VERSION);
      dxl_error = getLastRxPacketError(port_num1, PROTOCOL_VERSION);
      if dxl_comm_result ~= COMM_SUCCESS
          fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      elseif dxl_error ~= 0
          fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
      end

      % Read Dynamixel#2 and 7 present position
      dxl2_present_position = read2ByteTxRx(port_num2, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_PRESENT_POSITION);
      if getLastTxRxResult(port_num2, PROTOCOL_VERSION) ~= COMM_SUCCESS
            printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num2, PROTOCOL_VERSION));
      elseif getLastRxPacketError(port_num2, PROTOCOL_VERSION) ~= 0
            printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num2, PROTOCOL_VERSION));
      end

      dxl7_present_position = read2ByteTxRx(port_num2, PROTOCOL_VERSION, DXL7_ID, ADDR_MX_PRESENT_POSITION);
      if getLastTxRxResult(port_num2, PROTOCOL_VERSION) ~= COMM_SUCCESS
            printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num2, PROTOCOL_VERSION));
      elseif getLastRxPacketError(port_num2, PROTOCOL_VERSION) ~= 0
            printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num2, PROTOCOL_VERSION));
      end
      
      % Read Dynamixel#3 and 8 present position
      dxl3_present_position = read2ByteTxRx(port_num3, PROTOCOL_VERSION, DXL3_ID, ADDR_MX_PRESENT_POSITION);
      if getLastTxRxResult(port_num3, PROTOCOL_VERSION) ~= COMM_SUCCESS
            printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num3, PROTOCOL_VERSION));
      elseif getLastRxPacketError(port_num3, PROTOCOL_VERSION) ~= 0
            printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num3, PROTOCOL_VERSION));
      end

      dxl8_present_position = read2ByteTxRx(port_num3, PROTOCOL_VERSION, DXL8_ID, ADDR_MX_PRESENT_POSITION);
      if getLastTxRxResult(port_num3, PROTOCOL_VERSION) ~= COMM_SUCCESS
            printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num3, PROTOCOL_VERSION));
      elseif getLastRxPacketError(port_num3, PROTOCOL_VERSION) ~= 0
            printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num3, PROTOCOL_VERSION));
      end
      
      % Read Dynamixel#4 and 9 present position
      dxl4_present_position = read2ByteTxRx(port_num4, PROTOCOL_VERSION, DXL4_ID, ADDR_MX_PRESENT_POSITION);
      if getLastTxRxResult(port_num4, PROTOCOL_VERSION) ~= COMM_SUCCESS
            printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num4, PROTOCOL_VERSION));
      elseif getLastRxPacketError(port_num4, PROTOCOL_VERSION) ~= 0
            printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num4, PROTOCOL_VERSION));
      end

      dxl9_present_position = read2ByteTxRx(port_num4, PROTOCOL_VERSION, DXL9_ID, ADDR_MX_PRESENT_POSITION);
      if getLastTxRxResult(port_num4, PROTOCOL_VERSION) ~= COMM_SUCCESS
            printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num4, PROTOCOL_VERSION));
      elseif getLastRxPacketError(port_num4, PROTOCOL_VERSION) ~= 0
            printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num4, PROTOCOL_VERSION));
      end
      
      % Read Dynamixel#5 present position
      dxl5_present_position = read2ByteTxRx(port_num5, PROTOCOL_VERSION, DXL5_ID, ADDR_MX_PRESENT_POSITION);
      dxl_comm_result = getLastTxRxResult(port_num5, PROTOCOL_VERSION);
      dxl_error = getLastRxPacketError(port_num5, PROTOCOL_VERSION);
      if dxl_comm_result ~= COMM_SUCCESS
          fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      elseif dxl_error ~= 0
          fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
      end
      
      % Read Dynamixel#6 present position
      dxl6_present_position = read2ByteTxRx(port_num6, PROTOCOL_VERSION, DXL6_ID, ADDR_MX_PRESENT_POSITION);
      dxl_comm_result = getLastTxRxResult(port_num6, PROTOCOL_VERSION);
      dxl_error = getLastRxPacketError(port_num6, PROTOCOL_VERSION);
      if dxl_comm_result ~= COMM_SUCCESS
          fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      elseif dxl_error ~= 0
          fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
      end
      
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
