// Global Variables Here

// Global Variables
var Vnom: double;
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
begin
  SetAxisVoltageRef(0, 0, Vnom);
  SetAxisVoltageRef(0, 1, -Vnom);
  SetAxisVoltageRef(0, 2, Vnom);
  SetAxisVoltageRef(0, 3, -Vnom);
end;

// this procedure is called periodicaly (default: 40 ms)
procedure Control;
begin

  //State := 0;
  if KeyPressed(VK_ESCAPE) then State := 0;
  if KeyPressed(VK_UP) then State := 1;
  if KeyPressed(VK_DOWN) then State := 2;

  case State of
    0: Stop;
    1: Up;
    2: Down;
  else Stop;
  end;
end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  //SetThingPos(1, 1, 1, 1);
  Vnom := 3;
  State := 0;
  delta := 0.99;
end;
