MODULE Constants
    PERS string IP:="192.168.125.1";
    
    PERS num PORT_JOINTS_LOGGER_L:= 5010;
    PERS num PORT_JOINTS_LOGGER_R:= 5011;
    PERS num PORT_POSES_LOGGER_L:= 5012;
    PERS num PORT_POSES_LOGGER_R:= 5013;
    PERS num PORT_TORQUES_LOGGER_L:= 5014;
    PERS num PORT_TORQUES_LOGGER_R:= 5015;    
    
    PERS num PORT_SERVER_L:= 5000;
    PERS num PORT_SERVER_R:= 5001;
    
    PERS num LOGGER_PERIOD:= 0.01;
    
    PERS bool TP_ENABLED:=FALSE;
    
    !Robot configuration	
    PERS tooldata currentTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    PERS wobjdata currentWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    
    PROC main()
        WHILE TRUE DO
        ENDWHILE
    ENDPROC
    
ENDMODULE