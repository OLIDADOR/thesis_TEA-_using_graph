type
  TJoints3D = record
    t1, t2, t3: double;
  end;

// Global Variables
var
  irobot, B4, B6: integer;
  theta: array[1..6] of double;
  Pos: TPoint3D;
  
  degs: double;

function Direct3D(t1, t2, t3: double): TPoint3D;
begin
  result.x := 0;
  result.y := 0;
  result.z := 0;
end;


function Inverse3D(target: TPoint3D): TJoints3D;
begin
  result.t1 := 0;
  result.t2 := 0;
  result.t3 := 0;
end;

procedure MatDemo;
var A, B, C, D: Matrix;
    v: double;
begin
  // Matriz só com zeros
  A := Mzeros(3, 3);

  // A[0,0] = 5.1
  Msetv(A, 0, 0, 5.1);

  // v = A[0,0]
  v := Mgetv(A, 0, 0);
  
  // B = matriz identidade de dimensão 3
  B := Meye(3);
  
  // C = B^-1
  C := Minv(B);

  // C = A * B
  C := Mmult(A, B)

  // Transpor C
  C := Mtran(C);
  
  // D = A + B
  D := Madd(A , B);
  
  // D = A - B
  D := Msub(A , B);

  // Multiplicar todos os elementos de A por 5.2
  D := MmultReal(A, 5.2);
  
  // A = B * C' + D
  A := Madd(Mmult(B, Mtran(C)), D);

  Msetv(A, 1, 0, 5.1);

  MatrixToRange(15, 2, A);

  SetRCValue(19, 2, FloatToStr(MallNorm(A)));
  SetRCValue(20, 3, FloatToStr(Power(2, 0.5)));

end;


procedure Control;
var v: double;
begin
  MatDemo();
  SetRCValue(3, 8, FloatToStr(GetAxisPosDeg(0, 0)));
  //degs := degs + 1;

  if RCButtonPressed(2, 7) then begin
    degs := degs + 20;
    v := GetRCValue(2,8);
    v := v  + 1;
    SetRCValue(2, 8, FloatToStr(v));
  end;
  
  if degs > 90 then degs := 0;
  SetAxisPosRef(0, 0, rad(degs));

end;

procedure Initialize;
begin
degs := 0;
  irobot := 0;
  B4 := GetSolidIndex(irobot, 'B4');
  SetRCValue(2, 2, 'Ola');
end;
