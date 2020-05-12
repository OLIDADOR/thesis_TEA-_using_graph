unit Unit3;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, ExtCtrls, Grids,
  StdCtrls;

type

  Workstation = object
   private
     {private declarations}
   public
     {public declarations}
     var
     id:integer;
     node_id:integer;
     pos_X:Double;
     pos_Y:Double;
     isactive:integer;
     ws_id:integer;
   end;

   w_node=array of Workstation;

  { TForm3 }

  TForm3 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    Button3: TButton;
    LabeledEdit1: TLabeledEdit;
    PaintBox1: TPaintBox;
    StringGrid1: TStringGrid;
    procedure Button1Click(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure Button3Click(Sender: TObject);
    procedure FormPaint(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure PaintBox1Paint(Sender: TObject);
  private

  public
   wos:array of Workstation;
  end;

var
  Form3: TForm3;
  newlabel: TLabel;
  flag_nld:integer;

implementation
     uses
   unit2,unit1;
{$R *.lfm}

function check_array(l:array of integer; i:integer):integer;
var
l1:integer;
count:integer;
i_curr:integer;
aux:integer;
begin
 l1:=length(l);
 count:=0;
 for aux:=0 to l1-1 do
 begin
   i_curr:=l[aux];
   if i_curr=i then
   begin
   count:=count+1;
   end;
end;
 check_array:=count;
end;
{ TForm3 }

procedure TForm3.Button3Click(Sender: TObject);
begin
  form3.Hide;
  form2.show;
end;

procedure TForm3.Button1Click(Sender: TObject);
var
id,size1,aux1:integer;
begin
  id:=strtoint(LabeledEdit1.Text);
  size1:=length(wos);
  for aux1:=0 to size1-1 do
  begin
    if wos[aux1].id=id then
    begin
      wos[aux1].isactive:=1;
    end;
  end;
  invalidate;
end;

procedure TForm3.Button2Click(Sender: TObject);
var
id,size1,aux1:integer;
begin
  id:=strtoint(LabeledEdit1.Text);
  size1:=length(wos);
  for aux1:=0 to size1-1 do
  begin
    if wos[aux1].id=id then
    begin
      wos[aux1].isactive:=0;
    end;
  end;
  invalidate;
end;

procedure TForm3.FormPaint(Sender: TObject);
var
  rc1,aux3,l1,act,nid,i_curr:integer;
  x,y:Double;
begin
  rc1:=StringGrid1.RowCount;
  if rc1>1 then
  begin
  for aux3:=1 to rc1-1 do
     begin
     StringGrid1.DeleteRow(1);
     end;
  end;
  l1:=length(wos);
  if l1>0 then begin
  for aux1:=0 to l1-1 do
  begin
  i_curr:=wos[aux1].id;
  x:=wos[aux1].pos_X;
  y:=wos[aux1].pos_y;
  act:=wos[aux1].isactive;
  nid:=wos[aux1].node_id;
  StringGrid1.InsertRowWithValues(1,[inttostr(i_curr) , floattostr(x), floattostr(Y), inttostr(act), inttostr(nid)]);
  end;
  end;

end;

procedure TForm3.FormShow(Sender: TObject);
var
  rc1,aux3,l1,size2,i_curr:integer;
  x,y:Double;
begin
  l1:=length(form1.full_nodelist);
  count:=1;
  for aux1:=0 to l1-1 do
  begin
  i_curr:=form1.full_nodelist[aux1].id;
  x:=form1.full_nodelist[aux1].pos_X;
  y:=form1.full_nodelist[aux1].pos_y;
  if check_array(form1.ws,i_curr)=1 then
  begin
       size2:=length(wos);
       setlength(wos,size2+1);
       wos[size2].id:=count;
       wos[size2].isactive:=0;
       wos[size2].pos_X:=x;
       wos[size2].pos_Y:=y;
       wos[size2].node_id:=i_curr;
       count:=count+1;
  end;
  flag_nld:=0;
  end;

end;

procedure TForm3.PaintBox1Paint(Sender: TObject);
var
  l1,l2,l3,id_l,aux1,aux2,aux3,id_i:integer;
  X_print,Y_print,X_print1,Y_print1:Longint;
begin
  Canvas := form3.PaintBox1.Canvas;
  l1:=length(form1.full_nodelist);
  if l1>0 then begin
   //Print Links
    Canvas.Pen.Width:=2;
    Canvas.Pen.Color:=clLime;
    for aux1:=0 to l1-1 do
        begin
          X_print:=round(form1.full_nodelist[aux1].pos_X*200);
          Y_print:=round(form1.full_nodelist[aux1].pos_y*200);
          l2:=length(form1.full_nodelist[aux1].links);
           for aux2:=0 to l2-1 do
           begin
              id_l:=form1.full_nodelist[aux1].links[aux2].node_to_link;
               for aux3:=0 to l1-1 do
                   begin
                     if form1.full_nodelist[aux3].id=id_l then
                     begin
                        X_print1:=round(form1.full_nodelist[aux3].pos_X*200);
                        Y_print1:=round(form1.full_nodelist[aux3].pos_y*200);
                        Canvas.Line(X_print, Y_print, X_print1, Y_print1);
                     end;
                   end;
           end;
        end;
  end;
   //Print Workstations
    Canvas.Pen.Width:=8;
    Canvas.Pen.Color:=clRed;
   l3:=length(wos);
   if l3>0 then begin
   for aux1:=0 to l3-1 do
       begin
          id_i:=wos[aux1].id;
          X_print:=round(wos[aux1].pos_X*200);
          Y_print:=round(wos[aux1].pos_y*200);
          if wos[aux1].isactive=0 then
          begin
             Canvas.Pen.Color:=clRed;
          end
          else if wos[aux1].isactive=1 then
          begin
             Canvas.Pen.Color:=clblue;
          end;
          if flag_nld=0 then
           begin
             newlabel := TLabel.Create(Form3);
             newlabel.name := 'nlad'+inttostr(id_i);
             newlabel.caption := inttostr(id_i);
             newlabel.left := X_print+560-15;
             newlabel.Top := y_print+15-18;
             newlabel.visible := true;
             newlabel.parent := Form3;
           end;
          Canvas.Rectangle (X_print-1,Y_print-1,X_print+1,Y_print+1);
       end;
       flag_nld:=1;
   end;
end;

end.

