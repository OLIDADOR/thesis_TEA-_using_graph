unit Unit1;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, Grids, ExtCtrls,
  StdCtrls;

type

  { TForm5 }

  TForm5 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    Button3: TButton;
    LabeledEdit1: TLabeledEdit;
    LabeledEdit2: TLabeledEdit;
    PaintBox1: TPaintBox;
    StringGrid1: TStringGrid;
    StringGrid2: TStringGrid;
    procedure Button1Click(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure PaintBox1Paint(Sender: TObject);
  private

  public

  end;

var
  Form5: TForm5;
  lc2:integer;
  lc3:integer;
  aux4:integer;
  aux3:integer;
  flag_grid:integer;
  R:integer;
  possX:String;
  possY:String;
  n_links:String;
  id:String;
  l_done1:array of integer;
  l_curr:integer;
  n1_s:string;
  n2_s:string;
  dist_s:string;
  l4:integer;
  l5:integer;
  X_print:LongInt;
  Y_print:LongInt;
  X_print1:LongInt;
  Y_print1:LongInt;
  ND:Double;
  l_c:integer;
  NC1:integer;
  NC2:integer;
implementation
uses
   unit1_Dall,unit2,unit3;

{$R *.lfm}
function Get_Distance_ns(n1:integer;n2:integer; nodelist:a_node):Double;
  var
    n1_int:integer;
    n2_int:integer;
    l4:integer;
    aux4:integer;
    n_curr:integer;
    x1:Double;
    y1:Double;
    x2:Double;
    y2:Double;
    d_x:Double;
    d_y:Double;
    aux1:Double;
    aux2:Double;
    aux3:Double;
  begin
      n1_int:=n1;
      n2_int:=n2;
      l4:=length(nodelist);
  if n1_int>0 then
  begin
  if l4>0 then
  begin
  for aux4:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux4].id;
     if n_curr=n1_int then
     begin
     x1:=nodelist[aux4].posrealX;
     y1:=nodelist[aux4].posrealY;
     end
     else if n_curr=n2_int then
     begin
      x2:=nodelist[aux4].posrealX;
      y2:=nodelist[aux4].posrealY;
     end;
  end;
   d_x:=x2-x1;
   d_y:=y2-y1;
   aux2:=d_x*d_x;
   aux3:=d_y*d_y;
   aux1:=aux2+aux3;
   Get_Distance_ns:=sqrt(aux1);
   //Get_Distance:=d_x;
end;
end;
end;

function check_existance (e:integer;array_tc:array of integer):integer;
var
    l4:integer;
    aux4:integer;
    count:integer;

begin
  l4:=length(array_tc);
  count:=0;
  if l4>0 then
  begin
  for aux4:=0 to l4-1 do
  begin
    if e=array_tc[aux4] then
    begin
         count:=count+1;
    end;
  end;
end;
   check_existance:=count;
end;




procedure Recalc_Dist(nodelist:a_node);
var
   l4:integer;
   l5:integer;
   l6:integer;
   l7:integer;
   aux4:integer;
   aux5:integer;
   aux6:integer;
   n_curr:integer;
   l_curr:integer;
   n_link:integer;
   l_done:array of integer;
   dist:Double;
   nt:integer;
begin
  l4:=length(nodelist);
  if l4>0 then
  begin

  for aux4:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux4].id;
     l5:=length(nodelist[aux4].links);
     for aux5:=0 to l5-1 do
     begin
       l_curr:=nodelist[aux4].links[aux5].id_l;
       nt:=nodelist[aux4].links[aux5].node_to_link;
        if check_existance(l_curr,l_done)<1 then
           begin
             dist:=Get_Distance_ns(n_curr,nt,nodelist);
             nodelist[aux4].links[aux5].distance:=dist;
             l7:=length(l_done);
             setlength(l_done,l7+1);
             l_done[l7]:=l_curr;
           end;
     end;
  end;
  end;
end;




procedure Change_Dist(e:integer;d:double;nodelist:a_node);
var
   l4:integer;
   l5:integer;
   l6:integer;
   l7:integer;
   aux4:integer;
   aux5:integer;
   aux6:integer;
   n_curr:integer;
   l_curr:integer;
   n_link:integer;
   x1:Double;
   y1:Double;
   x2:Double;
   y2:Double;
   ind:integer;
   nx:Double;
   ny:Double;
   nt:integer;
   m:Double;
   b:Double;
   a:Double;
   bq:Double;
   c:Double;
   nx1:Double;
   nx2:Double;
   delta:Double;
begin
   l4:=length(nodelist);
  if l4>0 then
  begin

  for aux4:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux4].id;
     l5:=length(nodelist[aux4].links);
     for aux5:=0 to l5-1 do
     begin
       l_curr:=nodelist[aux4].links[aux5].id_l;
       nt:=nodelist[aux4].links[aux5].node_to_link;
       if l_curr=e then
       begin
        x1:=nodelist[aux4].posrealX;
        y1:=nodelist[aux4].posrealY;
        nodelist[aux4].links[aux5].distance:=d;
        for aux6:=0 to l4-1 do
        begin
          if nt=nodelist[aux6].id then
          begin
              x2:=nodelist[aux6].posrealX;
              y2:=nodelist[aux6].posrealY;
              ind:=aux6;
          end;
        end;
        if x1=x2 then
        begin
             if y2<y1 then
             begin
               ny:=dist+y2;
               nodelist[aux4].posrealY:=ny;
             end
             else
               begin
                ny:=dist+y1;
                nodelist[ind].posrealY:=ny;
               end;
        end
        else if y1=y2 then
        begin
           if x2<x1 then
             begin
               nx:=dist+x2;
               nodelist[aux4].posrealX:=nx;
             end
             else
               begin
                nx:=dist+x1;
                nodelist[ind].posrealX:=nx;
               end;
        end
        else
        begin
           m:=(y2-y1)/(x2-x1);
           b:=-m*x1+y1;
           a:=m+1;
           bq:=-4*(m*x1+x1);
           c:=(m*x1)*(m*x1)-(d*d);
           delta:=b*b-4*a*c;
           if delta<0 then
            else
            if delta=0 then begin
              nx:=-b/(2*a);
              ny:=m*nx+y1-m*x1;
              nodelist[ind].posrealX:=nx;
              nodelist[ind].posrealY:=ny;
            end else begin
              nx1:=(-b+sqrt(delta))/(2*a);
              nx2:=(-b-sqrt(delta))/(2*a);
             if ((nx1>=0) and (nx2>=0)) then
               begin
                if ((x2>=x1) and (nx1>=nx2)) then
                  begin
                   ny:=m*nx1+y1-m*x1;
                   nodelist[ind].posrealX:=nx1;
                   nodelist[ind].posrealY:=ny;
                  end
                else if ((x2>=x1) and (nx1<nx2)) then
                  begin
                   ny:=m*nx2+y1-m*x1;
                   nodelist[ind].posrealX:=nx2;
                   nodelist[ind].posrealY:=ny;
                  end
                else if ((x2<x1) and (nx1>=nx2)) then
                  begin
                   ny:=m*nx2+y1-m*x1;
                   nodelist[ind].posrealX:=nx2;
                   nodelist[ind].posrealY:=ny;
                  end
                else if ((x2<x1) and (nx1<nx2)) then
                  begin
                   ny:=m*nx1+y1-m*x1;
                   nodelist[ind].posrealX:=nx1;
                   nodelist[ind].posrealY:=ny;
                  end;
               end
              else if ((nx1>=0) and (nx2<0)) then
                begin
                   ny:=m*nx1+y1-m*x1;
                   nodelist[ind].posrealX:=nx1;
                   nodelist[ind].posrealY:=ny;
                 end
              else if ((nx1<0) and (nx2>=0)) then
                begin
                 ny:=m*nx2+y1-m*x1;
                 nodelist[ind].posrealX:=nx2;
                 nodelist[ind].posrealY:=ny;
                end;
            end;
          end;
               end;
             end;

end;
  end;
  end;

{ TForm5 }

procedure TForm5.FormShow(Sender: TObject);
begin
  lc2:=length(Form1.intersection_nodesXY);
  if lc2>0 then
  begin
  if flag_grid<1 then
  begin
  R:=1;
  for aux3:=0 to lc2-1 do
  begin
     possX:=FloatToStr(form1.intersection_nodesXY[aux3].posrealX);
     possY:=FloatToStr(form1.intersection_nodesXY[aux3].posrealY);
     id:=FloatToStr(form1.intersection_nodesXY[aux3].id);
     n_links:=FloatToStr(length(form1.intersection_nodesXY[aux3].links));
     StringGrid1.InsertRowWithValues(R,[id, possX, possY]);
     R:=R+1;
  end;
   R:=1;
   for aux3:=0 to lc2-1 do
     begin
       lc3:=length(Form1.intersection_nodesXY[aux3].links);
        for aux4:=0 to lc3-1 do
        begin
          l_curr:=form1.intersection_nodesXY[aux3].links[aux4].id_l;
           if check_existance(l_curr,l_done1)<1 then
           begin
              id:=FloatToStr(form1.intersection_nodesXY[aux3].links[aux4].id_l);
              n1_s:=FloatToStr(form1.intersection_nodesXY[aux3].id);
              n2_s:=FloatToStr(form1.intersection_nodesXY[aux3].links[aux4].node_to_link);
              dist_s:=FloatToStr(form1.intersection_nodesXY[aux3].links[aux4].distance);
              StringGrid2.InsertRowWithValues(R,[id, n1_s, n2_s, dist_s ]);
              R:=R+1;
              l7:=length(l_done1);
              setlength(l_done1,l7+1);
              l_done1[l7]:=l_curr;
           end;
        end;
     end;
  flag_grid:=flag_grid+1;
  end
  else
  begin
     for aux3:=1 to R-1 do
   begin
     StringGrid1.DeleteRow(1);
  end;
     R:=1;
    for aux3:=0 to lc2-1 do
    begin
       possX:=FloatToStr(form1.intersection_nodesXY[aux3].posrealX);
       possY:=FloatToStr(form1.intersection_nodesXY[aux3].posrealY);
       id:=FloatToStr(form1.intersection_nodesXY[aux3].id);
       n_links:=FloatToStr(length(form1.intersection_nodesXY[aux3].links));
       StringGrid1.InsertRowWithValues(R,[id, possX, possY]);
       R:=R+1;
    end;
    R:=1;
     for aux3:=0 to lc2-1 do
     begin
       lc3:=length(Form1.intersection_nodesXY[aux3].links);
        for aux4:=0 to lc3-1 do
        begin
          l_curr:=form1.intersection_nodesXY[aux3].links[aux4].id_l;
           if check_existance(l_curr,l_done1)<1 then
           begin
              id:=FloatToStr(form1.intersection_nodesXY[aux3].links[aux4].id_l);
              n1_s:=FloatToStr(form1.intersection_nodesXY[aux3].id);
              n2_s:=FloatToStr(form1.intersection_nodesXY[aux3].links[aux4].node_to_link);
              dist_s:=FloatToStr(form1.intersection_nodesXY[aux3].links[aux4].distance);
              StringGrid2.InsertRowWithValues(R,[id, n1_s, n2_s, dist_s ]);
              R:=R+1;
              l7:=length(l_done1);
              setlength(l_done1,l7+1);
              l_done1[l7]:=l_curr;
           end;
        end;
     end;
    flag_grid:=flag_grid+1;
  end;
  end;
end;

procedure TForm5.Button1Click(Sender: TObject);
begin
   l_c:=StrtoInt(form5.LabeledEdit1.Text);
   ND:=StrtoFloat(form5.LabeledEdit2.Text);
   Change_Dist(l_c,ND,form1.intersection_nodesXY);
   NC1:=form5.StringGrid1.RowCount;
   NC2:=form5.StringGrid2.RowCount;
   for aux3:=1 to NC1-1 do
         begin
           StringGrid1.DeleteRow(1);
        end;
   for aux3:=1 to NC2-1 do
         begin
           StringGrid2.DeleteRow(1);
        end;
   lc2:=length(Form1.intersection_nodesXY);
   if lc2>0 then
   begin
     R:=1;
  for aux3:=0 to lc2-1 do
  begin
     possX:=FloatToStr(form1.intersection_nodesXY[aux3].posrealX);
     possY:=FloatToStr(form1.intersection_nodesXY[aux3].posrealY);
     id:=FloatToStr(form1.intersection_nodesXY[aux3].id);
     n_links:=FloatToStr(length(form1.intersection_nodesXY[aux3].links));
     StringGrid1.InsertRowWithValues(R,[id, possX, possY]);
     R:=R+1;
  end;
   R:=1;
   for aux3:=0 to lc2-1 do
     begin
       lc3:=length(Form1.intersection_nodesXY[aux3].links);
        for aux4:=0 to lc3-1 do
        begin
          l_curr:=form1.intersection_nodesXY[aux3].links[aux4].id_l;
           if check_existance(l_curr,l_done1)<1 then
           begin
              id:=FloatToStr(form1.intersection_nodesXY[aux3].links[aux4].id_l);
              n1_s:=FloatToStr(form1.intersection_nodesXY[aux3].id);
              n2_s:=FloatToStr(form1.intersection_nodesXY[aux3].links[aux4].node_to_link);
              dist_s:=FloatToStr(form1.intersection_nodesXY[aux3].links[aux4].distance);
              StringGrid2.InsertRowWithValues(R,[id, n1_s, n2_s, dist_s ]);
              R:=R+1;
              l7:=length(l_done1);
              setlength(l_done1,l7+1);
              l_done1[l7]:=l_curr;
           end;
        end;
   end;
end;
      invalidate;
end;

procedure TForm5.FormCreate(Sender: TObject);
begin
end;

procedure TForm5.PaintBox1Paint(Sender: TObject);
begin
  Canvas:= form5.PaintBox1.Canvas;
  Canvas.Pen.Width:=8;
  Canvas.Pen.Color:=clGreen;
  l4:=length(form1.intersection_nodesXY);
  if l4>0 then
  begin
    for aux4:=0 to l4-1 do
    begin
     X_print:=round(form1.intersection_nodesXY[aux4].posrealX*926/form1.o_w);
     Y_print:=round(form1.intersection_nodesXY[aux4].posrealY*771/form1.o_h);
     Canvas.Rectangle (X_print-1,Y_print-1,X_print+1,Y_print+1);
    end;
    Canvas.Pen.Width:=2;
    Canvas.Pen.Color:=clLime;
    for aux4:=0 to l4-1 do
    begin
     l5:=length(form1.intersection_nodesXY[aux4].links);
     X_print:=round(form1.intersection_nodesXY[aux4].posrealX*926/form1.o_w);
     Y_print:=round(form1.intersection_nodesXY[aux4].posrealY*771/form1.o_h);
     for aux5:=0 to l5-1 do
     begin
       n_tlink:=form1.intersection_nodesXY[aux4].links[aux5].node_to_link;
       for aux6:=0 to l4-1 do
       begin
         if n_tlink=form1.intersection_nodesXY[aux6].id then
         begin
           X_print1:=round(form1.intersection_nodesXY[aux6].posrealX*926/form1.o_w);
           Y_print1:=round(form1.intersection_nodesXY[aux6].posrealY*771/form1.o_h);
           Canvas.Line(X_print, Y_print, X_print1, Y_print1);
         end;
       end;
     end;
    end;
  end;
end;

end.

