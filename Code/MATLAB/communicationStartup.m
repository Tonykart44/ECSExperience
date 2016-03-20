% Short script that initialises Roomba serial communication

roombaName = 'roombaName1' %Change this depending on used roomba
startup_RoombaUDP % Add neccesary files to path
IP=getIPRoomba(roombaName); % get IP adress of Roomba
comPort = initRoomba(IP); %IP adress of roomba (eg 192.168.1.101)

% Uncomment this line if LIDAR is going to be used
% LocalPort = initLidar(IP); %local address of LIDAR (eg 7070)