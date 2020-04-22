// Global Variables Here
var iZAxis, iXAxis, iYAxis: integer;

// this procedure is called periodicaly (default: 40 ms)
procedure Control;
var t: tcanvas;
begin
  if RCButtonPressed(1, 2) then
    SetSensorColor(0, 0, round(GetRCValue(1, 3)), round(GetRCValue(1, 4)), round(GetRCValue(1, 5)));
  if RCButtonPressed(3, 2) then
    SetAxisPosRef(0, iZAxis, GetRCValue(3, 3));
  if RCButtonPressed(4, 2) then
    SetAxisPosRef(0, iZAxis, GetRCValue(4, 3));
  if RCButtonPressed(6, 2) then begin
    SetAxisPosRef(0, iXAxis, GetRCValue(6, 3));
    SetAxisPosRef(0, iYAxis, GetRCValue(6, 4));
  end;
  if RCButtonPressed(7, 2) then begin
    SetAxisPosRef(0, iXAxis, 0);
    SetAxisPosRef(0, iYAxis, 0);
  end;
  t := GetSolidCanvas(0,0);
  t.brush.color := clwhite;
  t.textout(10,10, 'hello world!');
end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  iXAxis := GetAxisIndex(0, 'SlideX', 0);
  iYAxis := GetAxisIndex(0, 'SlideY', 0);
  iZAxis := GetAxisIndex(0, 'SlideZ', 0);
end;
