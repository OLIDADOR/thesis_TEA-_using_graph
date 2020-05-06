// 
// 
type
  TControlMode = (cmManual, cmScript);
  TRobotControls = record
    alpha, wt: double;
  end;

// Global Variables
var
  irobot, NumRobots: integer;
  t: double;
  sens_line: array[1..4] of double;
  ControlMode: TControlMode;
  RobotControls: TRobotControls;
  iLaser: integer;
  ODOWord1, ODOWord2: word;
  firstRay, lastRay: integer;
  Log: TStringList;
  LogOn: boolean;


function max(a, b: double): double;
begin
  if a >= b then result := a else result := b;
end;

function min(a, b: double): double;
begin
  if a <= b then result := a else result := b;
end;


procedure ManualControl(var RC: TRobotControls);
var alphamax, Wmax: double;
begin
  alphamax := rad(90);
  Wmax := 5;

  if KeyPressed(vk_down) then begin
    RC.wt := RC.wt - 0.5;
    RC.wt := max(RC.wt, -Wmax);
  end else if KeyPressed(vk_left) then begin
    RC.alpha := RC.alpha + rad(5);
    RC.alpha := min(RC.alpha, alphamax);
  end else if KeyPressed(vk_right) then begin
    RC.alpha := RC.alpha - rad(5);
    RC.alpha := max(RC.alpha, -alphamax);
  end else if KeyPressed(vk_up) then begin
    RC.wt := RC.wt + 0.5;
    RC.wt := min(RC.wt, Wmax);
  end;
end;


procedure ScriptControl(var RC: TRobotControls);
begin
  RC.alpha := 0;
  RC.wt := 0;
end;




procedure Control;
var StrPacket: TStringList;
    txt: string;
    i: integer;
    rgbacolor: trgbacolor;
    LaserValues: Matrix;

    posx, posy, posang: double;
    alpha_r, wt_r: double;
    newx, newy, newang: double;
begin
  if RCButtonPressed(6, 4) then begin
    newx := GetRCValue(7, 4);
    newy := GetRCValue(8, 4);
    newang := GetRCValue(9, 4);
    SetRobotPos(0, newx, newy, 0, rad(newang));
  end;
  
  t := t + ScriptPeriod();
  posx := getRobotX(0);
  posy := GetRobotY(0);
  posang := GetRobotTheta(0);
  SetRCValue(7, 2 ,format('%.3f',[posx]));
  SetRCValue(8, 2 ,format('%.3f',[posy]));
  SetRCValue(9, 2 ,format('%.3f',[deg(posang)]));

  alpha_r := GetAxisPos(0, 2);
  wt_r := GetAxisSpeed(0, 3);
  SetRCValue(11, 2, format('%.3f',[ deg(alpha_r)]));
  SetRCValue(12, 2, format('%.3f',[ wt_r]));


  {sens_esq_fr:=GetSensorVal(0,0);
  sens_esq_tr:=GetSensorVal(0,1);
  sens_dir_fr:=GetSensorVal(0,2);
  sens_dir_tr:=GetSensorVal(0,3);
  
  SetRCValue(7, 1 ,format('%.3g',[sens_esq_fr]));
  SetRCValue(8, 1 ,format('%.3g',[sens_esq_tr]));
  SetRCValue(7, 4 ,format('%.3g',[sens_dir_fr]));
  SetRCValue(8, 4 ,format('%.3g',[sens_dir_tr]));
  }

  for i:= 1 to 4 do begin
    sens_line[i] := GetSensorVal(0, i);
    SetRCValue(4, 3 + i,format('%.1f',[sens_line[i]]));
  end;

  if RCButtonPressed(2, 9) then begin // Start Log
    if log <> nil then Log.free;
    Log := TStringList.create;
    LogOn := true;
  end;
  
  if RCButtonPressed(2, 10) then begin // Stop Log
    LogOn := false;
  end;
  
  if RCButtonPressed(2, 11) then begin // Stop Log
    if log <> nil then Log.savetofile(GetRCText(2, 12));
  end;
  
  if LogOn then begin
    // x y theta enc1 enc2 Laser
    txt := format('%g %g %g  %d %d ', [posx, posy, posang, GetAxisOdo(0, 0), GetAxisOdo(0, 1)]);
    for i:= firstRay to LastRay do begin
      txt := txt + format('%g ', [Mgetv(LaserValues, i, 0)]);
    end;
    log.add(txt);
  end;

  if RCButtonPressed(2, 3) then begin
    ControlMode := cmManual;
    SetRCValue(2, 2, 'Manual');
  end else if RCButtonPressed(2, 4) then begin
    ControlMode := cmScript;
    SetRCValue(2, 2, 'Script');
  end;

  case ControlMode of
    cmManual: ManualControl(RobotControls);
    cmScript: ScriptControl(RobotControls);
  end;

  SetRCValue(11, 3 ,format('%.3f',[ deg(RobotControls.alpha)]));
  SetRCValue(12, 3 ,format('%.3f',[ RobotControls.wt]));
  
  SetAxisPosRef(irobot, 2, RobotControls.alpha);
  SetAxisSpeedRef(irobot, 3, RobotControls.wt);

end;

procedure Initialize;
begin
  iLaser := GetSensorIndex(0, 'ranger2d');
  firstRay := 0;
  lastRay := 359;
  t := 0;
  ControlMode := cmManual;
  SetRCValue(2, 2, 'Manual');
  Log := nil;
  LogOn := false;
end;
