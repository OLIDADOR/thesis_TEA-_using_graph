// 
// 

// Global Variables
var
  irobot, NumRobots: integer;
  Vnom: double;
  miss: integer;
  t: double;

procedure ControlAxis(i: integer; ref: double);
var teta, U: double;
begin
  teta := GetAxisPos(irobot, i);
  //writeln(floatTostr(teta[i]));
  U := (ref - teta) * 10;
  SetAxisVoltageRef(irobot, i, U);
end;

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

procedure EncodeDoubleArray(var StrPacket: TStringList; name: string; data: array of double);
var i: integer;
begin
  StrPacket.add(name);
  for i := 0 to GetArrayLength(data) - 1 do begin
    StrPacket.add(format('%.6g',[data[i]]));
  end;
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

function DecodeDoubleArray(var StrPacket: TStringList; name: string): array of double;
var i, j, size: integer;
begin
  setArrayLength(result, 0);
  i := StrPacket.indexof(name);
  if (i < 0)  then exit;
  size := 0;
  for j := i + 1 to StrPacket.count - 1 do begin
    if StrPacket[j] = '' then break;
    size := size + 1;
  end;
  
  if size = 0 then exit;

  setArrayLength(result, size);

  //for j := i + 1 to i + size do begin
  for j := 0 to size - 1 do begin
    result[j] := strtofloat(StrPacket[i + 1 + j]);
  end;
end;

procedure Control;
var StrPacket: TStringList;
    U1, U2: double;
    U: array of double;
    txt: string;
begin
  t := t + 0.04;
  AddTrailNode(0, sqrt(t)*sin(t), sqrt(t)*cos(t), 0.01);
  if t > 10 then begin
    ClearTrail(0);
    t := 0;
  end;

  SetAxisPosRef(irobot, 0, rad(80));
  SetRCValue(2, 2, format('%.2g %.2g',[U1, U2]));
  exit;
  //ControlAxis(0, rad(10));
  //ControlAxis(1, rad(10));

  //SetAxisVoltageRef(irobot, 0, Vnom)
  //SetAxisVoltageRef(irobot, 1, Vnom)
  StrPacket := TStringList.create;
  try
    EncodeDouble(StrPacket,'T1', GetAxisPos(0, 0));
    EncodeDouble(StrPacket,'T2', GetAxisPos(0, 1));
    EncodeDouble(StrPacket,'W1', GetAxisSpeed(0, 0));
    EncodeDouble(StrPacket,'W2', GetAxisSpeed(0, 1));

    WriteUDPData('127.0.0.1', 9810, StrPacket.text);

    StrPacket.text := ReadUDPData();
    txt := StrPacket.text;
    
    // Read Motor Voltage
    U1 := DecodeDoubleDef(StrPacket, 'U1', 0);
    U2 := DecodeDoubleDef(StrPacket, 'U2', 0);
    SetAxisVoltageRef(irobot, 0, U1);
    SetAxisVoltageRef(irobot, 1, U2);
    //SetAxisTorqueRef(0, 0, 10);
    //ControlAxis(0, rad(U1));
    //ControlAxis(1, rad(U2));
    SetRCValue(2, 2, format('%.2g %.2g',[U1, U2]));

  finally
    StrPacket.free;
  end;
end;

procedure Initialize;
begin
  t := 0;
  Vnom := 3;
  irobot := 0;
  NumRobots := 2;
  miss := 0;
  SetRCValue(2, 2, 'Ola');
  writeln(format('%d %s',[123, 'ola']));
end;
