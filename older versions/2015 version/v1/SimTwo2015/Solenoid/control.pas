// Global Variables Here
var is1: integer;

// this procedure is called periodicaly (default: 40 ms)
procedure Control;
begin
  if RCButtonPressed(3, 2) then begin
    SetGlobalSensorVin(is1, 1);
  end else if RCButtonPressed(3, 3) then begin
    SetGlobalSensorVin(is1, 0);
  end;

end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  ClearButtons();
  is1 := GetGlobalSensorIndex('solenoid1');
end;
