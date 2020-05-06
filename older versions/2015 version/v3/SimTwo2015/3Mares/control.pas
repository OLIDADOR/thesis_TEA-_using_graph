// Global Variables Here

// Global Variables
var Vnom: double;

//procedure ManualControl(out V, W, Vz: double);
procedure ManualControl();
var V, W, Vz, V1, V2: double;
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

  V1 := V * Vmax - W * Wmax;
  V2 := V * Vmax + W * Wmax;

  if KeyPressed(ord('1')) then begin
    Vz := -0.5;
  end else if KeyPressed(ord('2')) then begin
    Vz := 0;
  end else if KeyPressed(ord('3')) then begin
    Vz := 0.5;
  end;

  SetAxisVoltageRef(0, 0, Vnom*V1);
  SetAxisVoltageRef(0, 1, Vnom*V2);
  SetAxisVoltageRef(0, 2, Vnom*Vz);
  SetAxisVoltageRef(0, 3, Vnom*Vz);

end;


// this procedure is called periodicaly (default: 40 ms)
procedure Control;
begin
  ManualControl();
end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  Vnom := 30;
end;
