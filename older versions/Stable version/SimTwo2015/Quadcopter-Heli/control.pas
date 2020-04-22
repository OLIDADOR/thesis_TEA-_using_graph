// Global Variables Here

// Global Variables
var Vnom, Vheli, Vup: double;
    State: byte;
    delta: double;

procedure Stop;
begin
  SetAxisVoltageRef(0, 0, 0);
  SetAxisVoltageRef(0, 1, 0);
  SetAxisVoltageRef(0, 2, 0);
  SetAxisVoltageRef(0, 3, 0);
end;


procedure Down;
begin
  SetAxisVoltageRef(0, 0, -Vnom);
  SetAxisVoltageRef(0, 1, Vnom);
  SetAxisVoltageRef(0, 2, -Vnom);
  SetAxisVoltageRef(0, 3, Vnom);
end;

procedure Up;
var z, v: double;
begin
  z := GetSolidZ(0, 0);
  Vup := 2e-1 * (0.5 - z);
  v := 0.67 * Vnom + Vup;
  SetAxisVoltageRef(0, 0, v);
  SetAxisVoltageRef(0, 1, -v);
  SetAxisVoltageRef(0, 2, v);
  SetAxisVoltageRef(0, 3, -v);
end;

// this procedure is called periodicaly (default: 40 ms)
procedure Control;
begin

  //State := 0;
  if KeyPressed(VK_ESCAPE) then begin
    State := 0;
    Vheli := 0;
  end;
    
  if KeyPressed(VK_UP) then State := 1;
  if KeyPressed(VK_DOWN) then State := 2;

  if KeyPressed(ord('H')) then Vheli := Vnom;
  if KeyPressed(ord('J')) then Vheli := 0;

  case State of
    0: Stop;
    1: Up;
    2: Down;
  else Stop;
  end;

  SetAxisVoltageRef(0, 4, Vheli);
end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  //SetThingPos(1, 1, 1, 1);
  Vnom := 3;
  State := 0;
  delta := 0.99;
end;
