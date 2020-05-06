// Global Variables Here
var UDPData: String;
var start_status : Integer;
var time_ticks : Double;
var theta_variations : Double;
var AxisName: array [0..20] of String;
var AxisNumber: array [0..20] of Integer;
var AxisPos: array [0..20] of Double;
var numberAxis : Integer;
var feetPenalty : Integer;

// This procedure parses the data received from the UDP connection
// (sent by Matlab)
procedure ParseUDPData;
var tempString: String;
begin
  while Length(UDPData) > 0 do begin
    tempString := Copy(UDPData, (Pos('<',UDPData)+1),(Pos('>',UDPData)-Pos('<',UDPData)-1));
    if tempString = 'Start' then begin
      time_ticks := 0;
      feetPenalty := 1;
      SetRobotPos(0,0,0,0.39,0);
      start_status := 1;
    end
    else begin
      if Pos(':',tempString) <> 0 then begin
        AxisName[numberAxis] := Copy(tempString, 1, Pos(':',tempString)-1);
        AxisNumber[numberAxis] := StrToInt(Copy(tempString, Pos(':',tempString)+1, 1));
        AxisPos[numberAxis] := StrToFloat(Copy(tempString, Pos('=',tempString)+1, Length(tempString)));
      end
      else begin
        AxisName[numberAxis] := Copy(tempString, 1, Pos('=',tempString)-1);
        AxisNumber[numberAxis] := 0;
        AxisPos[numberAxis] := StrToFloat(Copy(tempString, Pos('=',tempString)+1, Length(tempString)-Pos('=',tempString)+1));
      end;
      numberAxis := numberAxis + 1;
    end;
  Delete(UDPData, Pos('<',UDPData),Pos('>',UDPData)-Pos('<',UDPData)+1);
  end;
end;


// this procedure is called periodicaly (default: 40 ms)
procedure Control;
var ReceivedUDPData : String;
var InitialDataPos : Integer;
var FinalDataPos : Integer;
var index: Integer;
var rightFootZ : Double;
var leftFootZ : Double;
begin
  ReceivedUDPData := ReadUDPData();
  InitialDataPos := Pos('<Data>', ReceivedUDPData);
  FinalDataPos := Pos('</Data>',ReceivedUDPData);
  if (InitialDataPos <> 0) AND (FinalDataPos <> 0) then begin
    UDPData:=Copy(ReceivedUDPData,InitialDataPos+6,(FinalDataPos-InitialDataPos-6));
    ParseUDPData;
    for index:=0 to numberAxis-1 do
    begin
      if GetAxisIndex(0,AxisName[index],AxisNumber[index]) <> -1 then
        SetAxisPosRef(0,GetAxisIndex(0,AxisName[index],AxisNumber[index]),Rad(AxisPos[index]));
    end;
    //Clear global variables
    numberAxis := 0;
  end;
  
  if (start_status <> 0) then begin
    time_ticks := time_ticks + 1;
    theta_variations := theta_variations + abs(GetRobotTheta(0));
    
    if time_ticks > 600 then begin //Time above 20 seconds
      writeUDPData('127.0.0.1', 19000, '<Fitness><Fit1>'+FloatToStr(GetRobotY(0))+'</Fit1><Fit2>'+FloatToStr(abs(GetRobotX(0)))+'</Fit2><Fit3>'+FloatToStr(theta_variations)+'</Fit3></Fitness>');
      start_status := 0;
    end
    else if GetSolidZ(0,GetSolidIndex(0,'head')) < 0.4 then begin
      writeUDPData('127.0.0.1', 19000, '<Fitness><Fit1>'+FloatToStr(-GetRobotY(0)+0.36)+'</Fit1><Fit2>'+FloatToStr(abs(GetRobotX(0)))+'</Fit2><Fit3>'+FloatToStr(theta_variations)+'</Fit3></Fitness>');
      start_status := 0;
    end
    else begin
      //Send current angles to control
      writeUDPData('127.0.0.1', 19000, '<Ticks>'+
                                       '<Time>'+FloatToStr(time_ticks)+'</Time>'+
                                       '<LeftHip_x>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'left_hip',0)))+'</LeftHip_x>'+
                                       '<LeftHip_y>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'left_hip',1)))+'</LeftHip_y>'+
                                       '<LeftKnee>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'left_knee',0)))+'</LeftKnee>'+
                                       '<LeftAnkle_x>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'left_ankle',0)))+'</LeftAnkle_x>'+
                                       '<LeftAnkle_y>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'left_ankle',1)))+'</LeftAnkle_y>'+
                                       '<RightHip_x>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'right_hip',0)))+'</RightHip_x>'+
                                       '<RightHip_y>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'right_hip',1)))+'</RightHip_y>'+
                                       '<RightKnee>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'right_knee',0)))+'</RightKnee>'+
                                       '<RightAnkle_x>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'right_ankle',0)))+'</RightAnkle_x>'+
                                       '<RightAnkle_y>'+FloatToStr(GetAxisPos(0,GetAxisIndex(0,'right_ankle',1)))+'</RightAnkle_y>'+
                                       '</Ticks>');
    end;
  end;
  
end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  numberAxis := 0;
  start_status:= 0;
  time_ticks:=0;
  theta_variations := 0;
end;
