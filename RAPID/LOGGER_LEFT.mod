MODULE LOGGER

!////////////////
!GLOBAL VARIABLES
!////////////////
!PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;

PERS string IP;
PERS num PORT_LOGGER_L;
PERS num LOGGER_PERIOD;

!Robot configuration	
PERS tooldata currentTool:=[TRUE,[[0,0,156],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
PERS wobjdata currentWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

PROC ServerCreateAndConnect(string ip, num port)
	VAR string clientIP;
	
	SocketCreate serverSocket;
	SocketBind serverSocket, ip, port;
	SocketListen serverSocket;
	TPWrite "LOGGER: Logger waiting for incomming connections ...";
	WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
		SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
		IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
			TPWrite "LOGGER: Problem serving an incomming connection.";
			TPWrite "LOGGER: Try reconnecting.";
		ENDIF
		 !Wait 0.5 seconds for the next reconnection
		 WaitTime 0.5;
	ENDWHILE
	TPWrite "LOGGER: Connected to IP " + clientIP;
ENDPROC

PROC main()
	VAR string data;
	VAR robtarget position;
	VAR jointtarget joints;
	VAR bool connected;

	VAR string time;
	VAR clock timer;

	time:= CTime();
    
	connected:=FALSE;
	ServerCreateAndConnect IP, PORT_LOGGER_L;	
    ClkStart timer;
	connected:=TRUE;
    
	WHILE TRUE DO
		
		!Cartesian Coordinates
		position := CRobT(\Tool:=currentTool \WObj:=currentWObj);
		data := "#0 ";
		data := data + NumToStr(ClkRead(timer),2) + " ";
		data := data + NumToStr(position.trans.x,1) + " ";
		data := data + NumToStr(position.trans.y,1) + " ";
		data := data + NumToStr(position.trans.z,1) + " ";
		data := data + NumToStr(position.rot.q1,3) + " ";
		data := data + NumToStr(position.rot.q2,3) + " ";
		data := data + NumToStr(position.rot.q3,3) + " ";
		data := data + NumToStr(position.rot.q4,3); !End of string	
		IF connected = TRUE THEN
			SocketSend clientSocket \Str:=data;
		ENDIF
		WaitTime LOGGER_PERIOD;
	
		!Joint Coordinates
		joints := CJointT();
		data := "#1 ";
		data := data + NumToStr(ClkRead(timer),2) + " ";
		data := data + NumToStr(joints.robax.rax_1,2) + " ";
		data := data + NumToStr(joints.robax.rax_2,2) + " ";
		data := data + NumToStr(joints.robax.rax_3,2) + " ";
		data := data + NumToStr(joints.robax.rax_4,2) + " ";
		data := data + NumToStr(joints.robax.rax_5,2) + " ";
		data := data + NumToStr(joints.robax.rax_6,2) + " "; 
        data:=data+NumToStr(joints.extax.eax_a,2);
        !End of string
		IF connected = TRUE THEN
			SocketSend clientSocket \Str:=data;
		ENDIF
		WaitTime LOGGER_PERIOD;
	ENDWHILE
	ERROR
    	IF ERRNO=ERR_SOCK_CLOSED THEN
    		TPWrite "LOGGER: Client has closed connection.";
        ELSEIF ERRNO=ERR_SOCK_TIMEOUT THEN
            TPWrite "LOGGER: Socket timed out.";
    	ELSE
    		TPWrite "LOGGER: Connection lost: Unknown problem.";
    	ENDIF
    	connected:=FALSE;
    	!Closing the server
    	SocketClose clientSocket;
    	SocketClose serverSocket;
    	!Reinitiate the server
    	ServerCreateAndConnect IP, PORT_LOGGER_L;
        ClkReset timer;
        ClkStart timer;
    	connected:= TRUE;
    	RETRY;
    
ENDPROC

ENDMODULE