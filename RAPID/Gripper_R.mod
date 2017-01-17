MODULE Gripper_L

    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !GLOBAL VARIABLES
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////

    !//PC communication
    VAR socketdev clientSocket;
    VAR socketdev serverSocket;
    VAR num instructionCode;
    VAR num params{10};
    VAR num nParams;

    PERS string ipController:="192.168.125.1";
    !robot default IP
    !PERS string ipController:= "127.0.0.1"; !local IP for testing in simulation
    VAR num serverPort:=5003;

    !//Correct Instruction Execution and possible return values
    VAR num ok;
    VAR bool should_send_res;
    CONST num SERVER_BAD_MSG:=0;
    CONST num SERVER_OK:=1;

    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !LOCAL METHODS
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////

    !//Method to parse the message received from a PC
    !// If correct message, loads values on:
    !// - instructionCode.
    !// - nParams: Number of received parameters.
    !// - params{nParams}: Vector of received params.
    PROC ParseMsg(string msg)
        !//Local variables
        VAR bool auxOk;
        VAR num ind:=1;
        VAR num newInd;
        VAR num length;
        VAR num indParam:=1;
        VAR string subString;
        VAR bool end:=FALSE;

        !//Find the end character
        length:=StrMatch(msg,1,"#");
        IF length>StrLen(msg) THEN
            !//Corrupt message
            nParams:=-1;
        ELSE
            !//Read Instruction code
            newInd:=StrMatch(msg,ind," ")+1;
            subString:=StrPart(msg,ind,newInd-ind-1);
            auxOk:=StrToVal(subString,instructionCode);
            ! ASG: set instructionCode here!
            IF auxOk=FALSE THEN
                !//Impossible to read instruction code
                nParams:=-1;
            ELSE
                ind:=newInd;
                !//Read all instruction parameters (maximum of 8)
                WHILE end=FALSE DO
                    newInd:=StrMatch(msg,ind," ")+1;
                    IF newInd>length THEN
                        end:=TRUE;
                    ELSE
                        subString:=StrPart(msg,ind,newInd-ind-1);
                        auxOk:=StrToVal(subString,params{indParam});
                        indParam:=indParam+1;
                        ind:=newInd;
                    ENDIF
                ENDWHILE
                nParams:=indParam-1;
            ENDIF
        ENDIF
    ENDPROC
        
    !//Handshake between server and client:
    !// - Creates socket.
    !// - Waits for incoming TCP connection.
    PROC ServerCreateAndConnect(string ip,num port)
        VAR string clientIP;

        SocketCreate serverSocket;
        SocketBind serverSocket,ip,port;
        SocketListen serverSocket;

        !! ASG: while "current socket status of clientSocket" IS NOT EQUAL TO the "client connected to a remote host"
        WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
            SocketAccept serverSocket,clientSocket\ClientAddress:=clientIP\Time:=WAIT_MAX;
            !//Wait 0.5 seconds for the next reconnection
            WaitTime 0.5;
        ENDWHILE
    ENDPROC

    FUNC string FormateRes(string clientMessage)
        VAR string message;
        
        message:=NumToStr(instructionCode,0);
        message:=message+" "+NumToStr(ok,0);
        message:=message+" "+ clientMessage;

        RETURN message;        
    ENDFUNC 

    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !//SERVER: Main procedure
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////

    PROC main()
        VAR progdisp progdisp1;
        
        !//Local variables
        VAR string receivedString;
        !//Received string
        VAR string sendString;
        !//Reply string
        VAR string addString;
        !//String to add to the reply.
        VAR bool connected;
        !//Client connected
        VAR bool reconnected;
        !//Reconnect after sending ack
        VAR bool reconnect;        

        !//Socket connection
        connected:=FALSE;
        ServerCreateAndConnect ipController,serverPort;
        connected:=TRUE;
        reconnect:=FALSE;
        
        !//Server Loop
        WHILE TRUE DO
            
            !//For message sending post-movement
            should_send_res:=TRUE;
            
            !//Initialization of program flow variables
            ok:=SERVER_OK;
            !//Has communication dropped after receiving a command?
            addString:="";
            !//Wait for a command
            SocketReceive clientSocket\Str:=receivedString\Time:=WAIT_MAX;
            ParseMsg receivedString;

            !//Correctness of executed instruction.
            reconnected:=FALSE;

            !//Execution of the command
            !---------------------------------------------------------------------------------------------------------------
            TEST instructionCode
            CASE 20:
                !Gripper Close
                IF nParams=0 THEN
                    Hand_GripInward;
                    ok:=SERVER_OK;

                    ! holdForce range = 0 - 20 N, targetPos = 0 - 25 mm, posAllowance = tolerance of gripper closure value
                ELSEIF nParams=2 THEN
                    Hand_GripInward\holdForce:=params{1}\targetPos:=params{2};
                    ok:=SERVER_OK;

                    ! Program won't wait until gripper completion or failure to move on.
                ELSEIF nParams=3 THEN
                    Hand_GripInward\holdForce:=params{1}\targetPos:=params{2}\NoWait;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 21:
                !Gripper Open
                IF nParams=0 THEN
                    Hand_GripOutward;
                    ok:=SERVER_OK;

                ELSEIF nParams=1 THEN
                    Hand_GripOutward\targetPos:=params{2};
                    ok:=SERVER_OK;

                    ! holdForce range = 0 - 20 N, targetPos = 0 - 25 mm, posAllowance = tolerance of gripper closure value
                ELSEIF nParams=2 THEN
                    Hand_GripOutward\holdForce:=params{1}\targetPos:=params{2};
                    ok:=SERVER_OK;

                    ! Program won't wait until gripper completion or failure to move on.
                ELSEIF nParams=3 THEN
                    Hand_GripOutward\holdForce:=params{1}\targetPos:=params{2}\NoWait;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 22:
                ! Initialize gripper with specified values

                ! calibrate only
                IF nParams=0 THEN
                    Hand_Initialize\Calibrate;
                    ok:=SERVER_OK;

                    ! set maxSpeed, holdForce, physicalLimit (0-25 mm), and calibrate                    
                ELSEIF nParams=3 THEN
                    Hand_Initialize\maxSpd:=params{1}\holdForce:=params{2}\phyLimit:=params{3}\Calibrate;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 23:
                ! Set Max Speed
                IF nParams=1 THEN
                    Hand_SetMaxSpeed params{1};
                    ! between 0-20 mm/s 
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 24:
                ! Set gripping force 
                IF nParams=0 THEN
                    Hand_SetHoldForce params{1};
                    ! between 0-20 Newtons
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 25:
                ! Move the gripper to a specified position 
                IF nParams=1 THEN
                    Hand_MoveTo params{1};
                    ! between 0-25 mm or 0-phyLimit if phyLimit is set in CASE 22
                    ok:=SERVER_OK;

                ELSEIF nParams=2 THEN
                    Hand_MoveTo params{1}\NoWait;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 26:
                !Get Gripper Width
                IF nParams=0 THEN
                    addString:=NumToStr(Hand_GetActualPos(),2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 29:
                ! Stop any action of the gripper (motors will lose power)
                IF nParams=0 THEN
                    Hand_Stop;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            DEFAULT:
                ok:=SERVER_BAD_MSG;
            ENDTEST
            !---------------------------------------------------------------------------------------------------------------
            !Compose the acknowledge string to send back to the client
            IF connected and reconnected=FALSE and SocketGetStatus(clientSocket)=SOCKET_CONNECTED and should_send_res THEN                                
                IF reconnect THEN
                    connected:=FALSE;
                    !//Closing the server
                    SocketClose clientSocket;
                    SocketClose serverSocket;
                    !Reinitiate the server
                    ServerCreateAndConnect ipController,serverPort;
                    connected:=TRUE;
                    reconnected:=TRUE;
                    reconnect:=FALSE;
                ELSE
                    SocketSend clientSocket\Str:=FormateRes(addString);
                ENDIF
            ENDIF
        ENDWHILE
        
    ERROR 
        ok:=SERVER_BAD_MSG;
        should_send_res:=FALSE;
        
        TEST ERRNO
            CASE ERR_HAND_WRONGSTATE:
                ok := SERVER_OK;
                should_send_res:=TRUE;
                RETRY;
            CASE ERR_SOCK_CLOSED:
                connected:=FALSE;
                !//Closing the server
                SocketClose clientSocket;
                SocketClose serverSocket;
                !//Reinitiate the server
                ServerCreateAndConnect ipController,serverPort;
                reconnected:=TRUE;
                connected:=TRUE;
                should_send_res:=TRUE;
                RETRY;
                
            CASE ERR_HAND_NOTCALIBRATED:
                SocketSend clientSocket\Str:=FormateRes( "ERR_HAND_NOTCALIBRATED: "+NumToStr(ERRNO,0));
                
                ! Gripper not calibrated.
                Hand_Initialize\Calibrate;
                RETRY;

            DEFAULT:
                SocketSend clientSocket\Str:=FormateRes("Default Error: "+NumToStr(ERRNO,0));
                connected:=FALSE;
                !//Closing the server
                SocketClose clientSocket;
                SocketClose serverSocket;
                !//Reinitiate the server
                ServerCreateAndConnect ipController,serverPort;
                reconnected:=TRUE;
                connected:=TRUE;
                RETRY;
        ENDTEST
    ENDPROC
    
ENDMODULE