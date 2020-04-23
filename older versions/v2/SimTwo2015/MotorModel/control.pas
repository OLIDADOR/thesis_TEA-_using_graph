//
// 

// Global Variables
var
  irobot, NumRobots: integer;
  Vnom: double;
  Udef: double;

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
    U, w: double;
    txt: string;
begin
  StrPacket := TStringList.create;
  try
    EncodeDouble(StrPacket,'T1', GetAxisPos(0, 0));
    EncodeDouble(StrPacket,'W1', GetAxisSpeed(0, 0));

    //WriteUDPData('127.0.0.1', 9810, StrPacket.text);

    //StrPacket.text := ReadUDPData();
    txt := StrPacket.text;
    
    // Read Motor Voltage
    U := DecodeDoubleDef(StrPacket, 'U1', 0);
    
    if KeyPressed(ord('0')) then Udef := 0;
    if KeyPressed(ord('1')) then Udef := 12;
    if KeyPressed(ord('3')) then Udef := 3;

    SetAxisVoltageRef(irobot, 0, Udef);
    w := GetAxisOdo(0,0);
    HUDStrings.strings[0]:=format('%.4g',[w]);
    //SetAxisTorqueRef(0, 0, 10);
    //writeln(format('%.2g',[U]));
    SetSheetRCString(2,2,format('%.2g',[w]));

  finally
    StrPacket.free;
  end;
end;

procedure Initialize;
begin
  Vnom := 12;
  irobot := 0;
  Udef := 0;
  writeln(format('%d %s',[123, 'Init']));
  HUDStrings.clear;
  HUDStrings.add('Test');
end;
