// 
// 
type
  TControlMode = (cmManual, cmScript, cmRemote);
  TRobotControls = record
    Fh: double;
    V1, V2: double;
  end;
  
  TPartState = (psRed, psGreen, psBlue, psYellow, psBlank);
  TCellState = (msEmpty, msBusy, msFull);
  
// Global Variables
var
  irobot, NumRobots: integer;
  t,sens_esq_fr,sens_esq_tr,sens_dir_fr,sens_dir_tr,sens_line_cent,sens_line_dir: double;
  sens_line: array[1..4] of double;
  ControlMode: TControlMode;
  RobotControls: TRobotControls;

procedure EncodeInteger(var StrPacket: TStringList; name: string; data: integer);
begin
  StrPacket.add(name);
  StrPacket.add(format('%d',[data]));
  StrPacket.add('');
end;

procedure EncodeDouble(var StrPacket: TStringList; name: string; data: double);
begin
  StrPacket.add(name);
  StrPacket.add(format('%.6g',[data]));
  StrPacket.add('');
end;

procedure EncodeDoubleFmt(var StrPacket: TStringList; name: string; data: double; fmt: string);
begin
  StrPacket.add(name);
  StrPacket.add(format(fmt,[data]));
  StrPacket.add('');
end;


function DecodeDoubleDef(var StrPacket: TStringList; name: string; defval: double): double;
var i: integer;
begin
  result := defval;
  i := StrPacket.indexof(name);
  if (i < 0) or (i + 1 >= StrPacket.count) then exit;
  result := strtofloat(StrPacket[i+1]);
end;


procedure ManualControl(var RC: TRobotControls);
var V, W: double;
    Vmax, Wmax: double;
begin
  Vmax := 4;
  Wmax := 2;

  V := 0;
  W := 0;

  if KeyPressed(vk_down) then begin
    V := -1;
  end else if KeyPressed(vk_left) then begin
    W := 1;
  end else if KeyPressed(vk_right) then begin
    W := -1
  end else if KeyPressed(vk_up) then begin
    V := 1;
  end;

  RC.V1 := V * Vmax - W * Wmax;
  RC.V2 := V * Vmax + W * Wmax;

  if KeyPressed(ord('1')) then begin
    RC.Fh := 0.0;
  end else if KeyPressed(ord('2')) then begin
    RC.Fh := 0.0152;
  end else if KeyPressed(ord('3')) then begin
    RC.Fh := 0.04;
  end;

end;


procedure ScriptControl(var RC: TRobotControls);
begin
  RC.V1 := 0;
  RC.V2 := 0;
  RC.Fh := 0;
end;


procedure RemoteControl(var RC: TRobotControls);
var StrPacket: TStringList;
    txt: string;
    i, WhiteLineCount, IRCount: integer;
begin
  StrPacket := TStringList.create;
  try
    EncodeInteger(StrPacket,'Enc1', GetAxisOdo(0, 0));
    EncodeInteger(StrPacket,'Enc2', GetAxisOdo(0, 1));

    IRCount := 4;
    EncodeInteger(StrPacket,'IRCount', IRCount);

    WhiteLineCount := 4;
    EncodeInteger(StrPacket,'WhiteLineCount', WhiteLineCount);

    for i := 0 to IRCount-1 do begin
      EncodeDouble(StrPacket, 'I' + inttostr(i), GetSensorVal(0, i));
    end;

    for i := 0 to WhiteLineCount-1 do begin
      EncodeDouble(StrPacket, 'W' + inttostr(i), GetSensorVal(0, IRCount + i));
    end;

    WriteUDPData(GetRCText(2,7), 9810, StrPacket.text);

    while true do begin
      StrPacket.text := ReadUDPData();
      txt := StrPacket.text;
      if txt <> '' then break;
    end;

    // Read Motor Speed Reference
    RC.V1 := DecodeDoubleDef(StrPacket, 'V1', 0);
    RC.V2 := DecodeDoubleDef(StrPacket, 'V2', 0);
    RC.Fh := DecodeDoubleDef(StrPacket, 'Fh', 0);
  finally
    StrPacket.free;
  end;
end;


procedure Control;
var StrPacket: TStringList;
    txt: string;
    i: integer;
    rgbacolor: trgbacolor;

begin
  t := t + ScriptPeriod();

  SetRCValue(1, 1, inttostr(GetObstacleIndex('Machines_B_LED1')));
  SetObstacleColor(GetObstacleIndex('Machines_B_LED1'),0, 255, 0);
  rgbacolor := GetObstacleColor(GetObstacleIndex('Machines_B_LED1'));

  sens_esq_fr:=GetSensorVal(0,0);
  sens_esq_tr:=GetSensorVal(0,1);
  sens_dir_fr:=GetSensorVal(0,2);
  sens_dir_tr:=GetSensorVal(0,3);
  
  SetRCValue(7, 1 ,format('%.3g',[sens_esq_fr]));
  SetRCValue(8, 1 ,format('%.3g',[sens_esq_tr]));
  SetRCValue(7, 4 ,format('%.3g',[sens_dir_fr]));
  SetRCValue(8, 4 ,format('%.3g',[sens_dir_tr]));

  for i:= 1 to 4 do begin
    sens_line[i]:=GetSensorVal(0,3+i);
    SetRCValue(5, i,format('%.1g',[sens_line[i]]));
  end;

  if RCButtonPressed(2, 3) then begin
    ControlMode := cmManual;
    SetRCValue(2, 2, 'Manual');
  end else if RCButtonPressed(2, 4) then begin
    ControlMode := cmScript;
    SetRCValue(2, 2, 'Script');
  end else if RCButtonPressed(2, 5) then begin
    ControlMode := cmRemote;
    SetRCValue(2, 2, 'Remote');
  end;

  case ControlMode of
    cmManual: ManualControl(RobotControls);
    cmScript: ScriptControl(RobotControls);
    cmRemote: RemoteControl(RobotControls);
  end;

  SetAxisSpeedRef(irobot, 0, RobotControls.V1);
  SetAxisSpeedRef(irobot, 1, RobotControls.V2);
  SetAxisPosRef(irobot, 2, RobotControls.Fh);

end;

procedure Initialize;
begin
  t := 0;
  ControlMode := cmManual;
  SetRCValue(2, 2, 'Manual');
end;
