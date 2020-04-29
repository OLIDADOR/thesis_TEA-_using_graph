unit Utils;

{$mode objfpc}{$H+}

interface

uses Classes,StdCtrls;

procedure VxyToVVn(teta,vx,vy: double; out V,Vn: double);
procedure VVnToVxy(teta,v,vn: double; out Vx,Vy: double);

procedure ZeroMemory(Ptr: Pointer; Len: integer);
procedure CopyMemory(DPtr,SPtr: Pointer; Len: integer);


function strtofloatDef(s: string; def: double): double;
function EditToFloatDef(edit: TEdit; default: double): double;
procedure ParseString(s,sep: string; sl: TStrings);

function FMod(x,d: double): double;
function DiffAngle(a1,a2: double): double;
function Dist(x,y: double): double;
function DistPointInLine(x1, y1, x2, y2, x3, y3: Double): Double;
function DistPointToLine(x1, y1, x2, y2, x3, y3: Double): Double;
function ATan2(y,x: double): double;
function Rad(xw: double):double;
function NormalizeAngle(ang: double): double;
function Sign(a: double): double; //SM 2018

procedure RotateAndTranslate(out rx,ry: double; px,py,tx,ty,teta: double);
procedure RotateAndTranslate1(var rx,ry,rvx,rvy: double; px,py,vx,vy,tx,ty,teta,v,vn: double);
procedure calcNextBallXY(var rx,ry: double; px,py,vx,vy,tx,ty,teta: double;forwardStep:integer);

procedure NormalizeVector(var x,y: double);


implementation

uses sysutils;

var FirstTimeValSec: LongInt;

procedure VxyToVVn(teta,vx,vy: double; out V,Vn: double);
var ct,st: double;
begin
  ct:=cos(teta);
  st:=sin(teta);
  v:=vx*ct+vy*st;
  vn:=-vx*st+vy*ct;
end;

procedure VVnToVxy(teta,v,vn: double; out Vx,Vy: double);
var ct,st: double;
begin
  ct:=cos(teta);
  st:=sin(teta);
  vx:=v*ct-vn*st;
  vy:=v*st+vn*ct;
end;

procedure NormalizeVector(var x,y: double);
var d: double;
begin
  d:=Dist(x,y);
  if abs(d)<1e-6 then begin
    x:=1;
    y:=0;
  end else begin
    x:=x/d;
    y:=y/d;
  end;
end;

function strtofloatDef(s: string; def: double): double;
begin
  try
    result:=strtofloat(s);
  except
    result:=def;
  end;
end;

function EditToFloatDef(edit: TEdit; default: double): double;
begin
  if edit.text='*' then begin
    result:=default;
    edit.text:=Format('%.8g',[default]);
    exit;
  end;
  try
    result:=strtofloat(edit.text);
  except
    result:=default;
    edit.text:=Format('%.8g',[default]);
  end;
end;

procedure ParseString(s,sep: string; sl: TStrings);
var p,i,last: integer;
begin
  sl.Clear;
  last:=1;
  for i:=1 to length(s) do begin
    p:=Pos(s[i],sep);
    if p>0 then begin
      if i<>last then
        sl.add(copy(s,last,i-last));
      last:=i+1;
    end;
  end;
  if last<=length(s) then
    sl.add(copy(s,last,length(s)-last+1));
end;

// ---------------------------------------------------------
//     Math functions

function Dist(x,y: double): double;
begin
  result:=sqrt(x*x+y*y);
end;

function DistPointInLine(x1, y1, x2, y2, x3, y3: Double): Double;
var
    teta, alfa, yp1, d1_3: Double;
begin

  // Angulo da reta no mundo
  teta:= ATan2(y2 - y1, x2 - x1);

  // Angulo da reta que une o ponto (x1, y1) ao ponto (x3, y3)
  alfa:= ATan2(y3 - y1, x3 - x1) - teta;

  d1_3:= Dist(x3 - x1, y3 - y1);

  Result:= d1_3 * cos(alfa);

end;

function DistPointToLine(x1, y1, x2, y2, x3, y3: Double): Double;
var
    teta, alfa, yp1, d1_3: Double;
begin

  // Angulo da reta no mundo
  teta:= ATan2(y2 - y1, x2 - x1);

  // Angulo da reta que une o ponto (x1, y1) ao ponto (x3, y3)
  alfa:= ATan2(y3 - y1, x3 - x1) - teta;

  d1_3:= Dist(x3 - x1, y3 - y1);

  Result:= d1_3 * sin(alfa);

end;

function FMod(x,d: double): double;
begin
  result:=Frac(x/d)*d;
end;

function DiffAngle(a1,a2: double): double;
begin
  result:=a1-a2;
  if result<0 then begin
    result:=-FMod(-result,2*Pi);
    if result<-Pi then result:=result+2*Pi;
  end else begin
    result:=FMod(result,2*Pi);
    if result>Pi then result:=result-2*Pi;
  end;
end;

function ATan2(y,x: double): double;
var ax,ay: double;
begin
  ax:=Abs(x);
  ay:=Abs(y);

  if (ax<1e-10) and (ay<1e-10) then begin;
    result:=0.0;
    exit;
  end;
  if ax>ay then begin
    if x<0 then begin
      result:=ArcTan(y/x)+pi;
      if result>pi then result:=result-2*pi;
    end else begin
      result:=ArcTan(y/x);
    end;
  end else begin
    if y<0 then begin
      result:=ArcTan(-x/y)-pi/2
    end else begin
      result:=ArcTan(-x/y)+pi/2;
    end;
  end;
end;

function Rad(xw: double):double;
begin
  result:=xw*(pi/180);
end;

procedure RotateAndTranslate(out rx,ry: double; px,py,tx,ty,teta: double);
var vx,vy: double;
begin
// Rotacao do vector (px,py) do angulo teta seguida de
// Translacao segundo o vector (tx,ty)
  vx:=px*cos(teta)-py*sin(teta);
  vy:=px*sin(teta)+py*cos(teta);
  rx:=vx+tx;
  ry:=vy+ty;
end;

procedure RotateAndTranslate1(var rx,ry,rvx,rvy: double; px,py,vx,vy,tx,ty,teta,v,vn: double);
var xrot,yrot: double;
begin
  //posicao
  xrot:=px*cos(teta)-py*sin(teta);
  yrot:=px*sin(teta)+py*cos(teta);
  rx:=xrot+tx;
  ry:=yrot+ty;
  //velocidade
  rvx:=(vx-v)*cos(teta)-(vy-vn)*sin(teta);
  rvy:=(vx-v)*sin(teta)+(vy-vn)*cos(teta);
end;

procedure calcNextBallXY(var rx, ry: double; px, py, vx, vy, tx, ty,
  teta: double; forwardStep: integer);
var xrot,yrot: double;
    x_next,y_next:double;
begin
  x_next:=px+vx*forwardStep*0.025;
  y_next:=py+vy*forwardStep*0.025;
  //posicao
  xrot:=x_next*cos(teta)-y_next*sin(teta);
  yrot:=x_next*sin(teta)+y_next*cos(teta);
  rx:=xrot+tx;
  ry:=yrot+ty;
end;

function NormalizeAngle(ang: double): double;
var a: double;
begin
  a:=FMod(ang+Pi,2*Pi);
  if a<0 then result:=a+Pi
  else result:=a-Pi;
end;

function Sign(a: double): double;
begin
  if (a < 0) then
    result := -1
  else
    result := 1;
end;

procedure ZeroMemory(Ptr: Pointer; Len: integer);
begin
  FillChar(Ptr^,len,0);
end;

procedure CopyMemory(DPtr,SPtr: Pointer; Len: integer);
begin
  Move(SPtr^,DPtr^,len);
end;


end.
