// Global Variables Here

// this procedure is called periodicaly (default: 40 ms)
procedure Control;
var vs: TPoint3d;
    A, B, C, R: TDMatrix;
begin
  //StartSolidFire(0,1);
  {if KeyPressed(vk_down) then begin
    vs := GetThingSize(1);
    SetThingSize(1, 0.9*vs.x, 0.9*vs.y, 0.9*vs.z);
  end else if KeyPressed(vk_up) then begin
    vs := GetThingSize(1);
    SetThingSize(1, vs.x/0.9, vs.y/0.9, vs.z/0.9);
  end;}
  
  A := RangeToMatrix(4, 2, 3, 3);
  B := RangeToMatrix(4, 6, 3, 3);
  C := Mzeros(3, 3);
  MSetv(C, 1, 1, 3.5);
  
  if RCButtonPressed(9, 1) then begin
    R := Minv(A);
    MatrixToRange(9, 2, R);
  end;

  if RCButtonPressed(9, 5) then begin
    MatrixToRange(9, 6, C);
  end;

  if RCButtonPressed(14, 1) then begin
    R := MAdd(A,B);
    MatrixToRange(14, 2, R);
  end;

  if RCButtonPressed(19, 1) then begin
    R := MMult(A,B);
    MatrixToRange(19, 2, R);
  end;

end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  SetThingPos(2, 0, 0, 1);
end;

