MODULE joints_logger

!////////////////
!GLOBAL VARIABLES
!////////////////
!PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;

PERS string IP;
PERS num PORT_JOINTS_LOGGER_L;
PERS tooldata currentTool;
PERS num LOGGER_PERIOD;

PROC ServerCreateAndConnect(string ip, num port)
	VAR string clientIP;
	
	SocketCreate serverSocket;
	SocketBind serverSocket, ip, port;
	SocketListen serverSocket;
	WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
		SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
		WaitTime 0.5;
	ENDWHILE
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
	ServerCreateAndConnect IP, PORT_JOINTS_LOGGER_L;	
    ClkStart timer;
	connected:=TRUE;
    
	WHILE TRUE DO
	
		!Joint Coordinates
		joints := CJointT();
		data := "#";
		data := data + NumToStr(ClkRead(timer),2) + " ";
		data := data + NumToStr(joints.robax.rax_1,2) + " ";
		data := data + NumToStr(joints.robax.rax_2,2) + " ";
		data := data + NumToStr(joints.robax.rax_3,2) + " ";
		data := data + NumToStr(joints.robax.rax_4,2) + " ";
		data := data + NumToStr(joints.robax.rax_5,2) + " ";
		data := data + NumToStr(joints.robax.rax_6,2) + " "; 
        data := data+NumToStr(joints.extax.eax_a,2);
        data := data + "!";
        !End of string
		IF connected = TRUE THEN
			SocketSend clientSocket \Str:=data;
		ENDIF

        WaitTime LOGGER_PERIOD;
	ENDWHILE
	ERROR
    	connected:=FALSE;
    	!Closing the server
    	SocketClose clientSocket;
    	SocketClose serverSocket;
    	!Reinitiate the server
    	ServerCreateAndConnect IP, PORT_JOINTS_LOGGER_L;
        ClkReset timer;
        ClkStart timer;
    	connected:= TRUE;
    	RETRY;
    
ENDPROC

ENDMODULE