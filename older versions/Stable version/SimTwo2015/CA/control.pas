// Global Variables Here
var vref, steer: double;

// this procedure is called periodicaly (default: 40 ms)
procedure Control;
var cnv: TCanvas;
begin
  if RCButtonPressed(3, 3) then begin
    vref := GetRCValue(3, 2);
    steer := rad(GetRCValue(7, 2));
  end;
  if RCButtonPressed(3, 4) then begin
    vref := 0;
  end;
  if RCButtonPressed(7, 4) then begin
    steer := 0;
  end;

  SetAxisSpeedRef(0, 0, vref);
  SetAxisSpeedRef(0, 1, -vref);

  SetAxisPosRef(0, 3, steer);
  SetAxisPosRef(0, 5, steer);

  SetRCValue(4, 2, format('%.2f', [GetAxisSpeed(0, 0)]));
  SetRCValue(5, 2, format('%.2f', [-GetAxisSpeed(0, 1)]));

  SetRCValue(8, 2, format('%.2f', [deg(GetAxisPos(0, 3))]));
  SetRCValue(9, 2, format('%.2f', [deg(GetAxisPos(0, 5))]));

  if RCButtonPressed(1, 1) then begin
    SolidCanvasClear(0, 0);
  end;
  cnv := GetSolidCanvas(0, 0);
  cnv.textout(0, 0, floattostr(vref));
  //cnv.ellipse(10,10,100,100);
end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  
end;
