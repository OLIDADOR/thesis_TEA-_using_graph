unit unit1_Dall;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, ExtCtrls, StdCtrls,
  ExtDlgs, ActnList;

type

   link = object
   private
     {private declarations}
   public
     {public declarations}
     var
     id_l:integer;
     node_to_link:LongInt;
     distance:Double;
   end;
  node = object
   private
     {private declarations}
   public
     {public declarations}
     constructor Init;
     var
     id:integer;
     posnX:LongInt;
     posnY:LongInt;
     posrealX:Double;
     posrealY:Double;
     iscritical:integer;
     links:array of link;
   end;

   a_node=array of node;


  { TForm1 }

  TForm1 = class(TForm)
    Add_toggle: TToggleBox;
    Button1: TButton;
    Button2: TButton;
    Button3: TButton;
    heigth: TLabeledEdit;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    map: TImage;
    Label1: TLabel;
    Label2: TLabel;
    OpenPictureDialog1: TOpenPictureDialog;
    RadioButton1: TRadioButton;
    RadioButton2: TRadioButton;
    RadioGroup1: TRadioGroup;
    RadioGroup2: TRadioGroup;
    ToggleBox1: TToggleBox;
    vel_nom11: TLabeledEdit;
    width1: TLabeledEdit;
    procedure Add_toggleClick(Sender: TObject);
    procedure Button1Click(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure Button3Click(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormPaint(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure Label3Click(Sender: TObject);
    procedure mapClick(Sender: TObject);
    procedure mapPaint(Sender: TObject);
    procedure RadioGroup1Click(Sender: TObject);
    procedure scaleChange(Sender: TObject);
    procedure ToggleBox1Change(Sender: TObject);
  private

  public
   intersection_nodesXY: array of node;
   aux:integer;
   f_scale:Double;
   o_w:Double;
   o_h:Double;
   vel_nom:Double;
  end;

var
  Form1: TForm1;
  node_temp: node;
  node_temp_d: node;
  pointX:LongInt;
  pointY:LongInt;
  pointg:TPoint;
  pointl:TPoint;
  posX:LongInt;
  posY:LongInt;
  x_temp:LongInt;
  y_temp:LongInt;
  aux2:integer;
  l:integer;
  lc:integer;
  vCntr:integer;
  id_r:integer;
  PixelColor :Tcolor;
  dist:integer;
  c_count:integer;
  p:double;
implementation
    uses
   unit2;
{$R *.lfm}

constructor node.Init;
begin
   id:=0;
   posnX:=0;
   posnY:=0;
end;


function get_max_id_link(nodelist:a_node):integer;
var
  l1:integer;
  l2:integer;
  aux1:integer;
  aux2:integer;
  i_max:integer;
  i_curr:integer;

  begin
      l1:=length(nodelist);
      i_max:=0;
      for aux1:=0 to l1-1 do
      begin
       l2:=length(nodelist[aux1].links);
       for aux2:=0 to l2-1 do
       begin
          i_curr:=nodelist[aux1].links[aux2].id_l;
          if i_max<i_curr then
          begin
            i_max:=i_curr;
          end;
       end;
      end;
      get_max_id_link:=i_max;
  end;

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
Function check_if_link_exists (n1:integer; n2:integer; nodelist:a_node):integer;

var
   l1:integer;
   l2:integer;
   n_curr:integer;
   ntl:integer;
   count:integer;
begin
    l1:=length(nodelist);
    count:=0;
  for aux4:=0 to l1-1 do
  begin
     n_curr:=nodelist[aux4].id;
     if n_curr=n1 then
     begin
      l2:=length(nodelist[aux4].links);
      for aux5:=0 to l2-1 do
      begin
        ntl:=nodelist[aux4].links[aux5].node_to_link;
        if ntl=n2 then
        begin
           count:=1;
        end;
      end;
     end;
  end;
  check_if_link_exists:=count;
end;

procedure create_link_between (n1:integer; n2:integer; nodelist:a_node);
var
  dist:Double;
  id:integer;
  l1:integer;
  l2:integer;
  n_curr:integer;
  c:integer;
begin
  c:=check_if_link_exists(n1,n2,nodelist);
  if c=0 then
  begin
  id:=get_max_id_link(nodelist);
  dist:=Get_Distance_ns(n1,n2,nodelist);
  l1:=length(nodelist);
  for aux4:=0 to l1-1 do
  begin
     n_curr:=nodelist[aux4].id;
     if n_curr=n1 then
     begin
      l2:=length(nodelist[aux4].links);
      setlength(nodelist[aux4].links,l2+1);
      nodelist[aux4].links[l2].id_l:=id+1;
      nodelist[aux4].links[l2].distance:=dist;
      nodelist[aux4].links[l2].node_to_link:=n2;
     end
     else if n_curr=n2 then
     begin
      l2:=length(nodelist[aux4].links);
      setlength(nodelist[aux4].links,l2+1);
      nodelist[aux4].links[l2].id_l:=id+1;
      nodelist[aux4].links[l2].distance:=dist;
      nodelist[aux4].links[l2].node_to_link:=n1;
     end;
  end;
  end;
end;

function get_node_xr(range:integer;nodelist:a_node;xn:longint;yn:longint):Integer;
  var
    aux0:integer;
    l1:integer;
    x_c:longint;
    y_c:longint;
    i_curr:integer;
    x_curr:integer;
    flag_c:integer;
begin
   l1:=length(nodelist);
   i_curr:=0;
   x_curr:=1000;
   for aux0:=0 to l1-1 do
   begin
     x_c:=nodelist[aux0].posnX;
     y_c:=nodelist[aux0].posnY;
     if ((x_c>xn) and ((y_c<=yn+range) and (y_c>=yn-range)) and (x_c<>xn)) then
     begin
       if x_c<x_curr then
       begin
        i_curr:=aux0+1;
        x_curr:=x_c
       end;
     end;
   end;
    get_node_xr:=i_curr;
end;

function get_node_xl(range:integer;nodelist:a_node;xn:longint;yn:longint):Integer;
  var
    aux0:integer;
    l1:integer;
    x_c:longint;
    y_c:longint;
    i_curr:integer;
    x_curr:integer;
    flag_c:integer;
begin
   l1:=length(nodelist);
   i_curr:=0;
   x_curr:=0;
   for aux0:=0 to l1-1 do
   begin
     x_c:=nodelist[aux0].posnX;
     y_c:=nodelist[aux0].posnY;
     if ((x_c<xn) and ((y_c<=yn+range) and (y_c>=yn-range)) and (x_c<>xn)) then
     begin
     if x_c>x_curr then
       begin
        i_curr:=aux0+1;
        x_curr:=x_c;
       end;
     end;
   end;
    get_node_xl:=i_curr;
end;

function get_node_yr(range:integer;nodelist:a_node;xn:longint;yn:longint):Integer;
  var
    aux0:integer;
    l1:integer;
    x_c:longint;
    y_c:longint;
    i_curr:integer;
    y_curr:integer;
    flag_c:integer;
begin
   l1:=length(nodelist);
   i_curr:=0;
   y_curr:=1000;
   for aux0:=0 to l1-1 do
   begin
     x_c:=nodelist[aux0].posnX;
     y_c:=nodelist[aux0].posnY;
     if ((y_c>yn) and ((x_c<=xn+range) and (x_c>=xn-range)) and (y_c<>yn)) then
     begin
      if y_c<y_curr then
       begin
        i_curr:=aux0+1;
        y_curr:=y_c;
       end;
     end;
   end;
    get_node_yr:=i_curr;
end;

function get_node_yl(range:integer;nodelist:a_node;xn:longint;yn:longint):Integer;
  var
    aux0:integer;
    l1:integer;
    x_c:longint;
    y_c:longint;
    i_curr:integer;
    y_curr:integer;
    flag_c:integer;
begin
   l1:=length(nodelist);
   i_curr:=0;
   y_curr:=0;
   for aux0:=0 to l1-1 do
   begin
     x_c:=nodelist[aux0].posnX;
     y_c:=nodelist[aux0].posnY;
     if ((y_c<yn) and ((x_c<=xn+range) and (x_c>=xn-range)) and (y_c<>yn)) then
     begin
     if y_c>y_curr then
       begin
        i_curr:=aux0+1;
        y_curr:=y_c;
       end;
     end;
   end;
    get_node_yl:=i_curr;
end;

function get_node_id(range:integer; x:longint; y:longint;nodelist:a_node;xn:longint;yn:longint):Integer;
  var
    aux0:integer;
    l1:integer;
    x_c:longint;
    y_c:longint;
    flag_c:integer;
begin
   l1:=length(nodelist);
   flag_c:=0;
   for aux0:=0 to l1-1 do
   begin
     x_c:=nodelist[aux0].posnX;
     y_c:=nodelist[aux0].posnY;
     if (((x_c<=x+range) and (x_c>=x-range)) and ((y_c<=y+range) and (y_c>=y-range)) and ((x_c<>xn) or (y_c<>yn))) then
     begin
       get_node_id:=aux0+1;
       flag_c:=1;
       exit;
     end;
   end;
   if flag_c=0 then
   begin
    get_node_id:=0;
   end;
end;

function check_colour(range:integer ;x1: longint; y1:longint; x2:longint; y2:longint; map:TImage):Integer;
   var
   temp_y:longint;
   temp_x:longint;
   diff1:integer;
   aux1:integer;
   aux2:integer;
   x_a:integer;
   y_a:integer;
   cr:string;
   count:Integer;
begin
  count:=0;
  if (((x1<=x2+range) and (x1>=x2-range)) and (y1>y2)) then
  begin
     diff1:=y1-y2;
     temp_y:=y1;
     if range<=x1 then
     begin
     x_a:=x1-range;
     end
     else
     begin
     x_a:=0;
     end;
     count:=0;
     for aux1:=0 to diff1 do
     begin
          for aux2:=x_a to (x1+range) do
          begin
             PixelColor:=map.Canvas.Pixels[aux2,y1+aux1];
             cr:=ColorToString(PixelColor);
             if cr='clBlack' then
             begin
               count:=count+1;
               check_colour:=count;
             end;
          end;
     end;
  end
   else if (((x1<=x2+range) and (x1>=x2-range)) and (y1<y2)) then
   begin
   diff1:=y2-y1;
   if range<=x1 then
     begin
       x_a:=x1-range;
     end
     else
     begin
       x_a:=0;
     end;
       count:=0;
       for aux1:=0 to diff1 do
       begin
          for aux2:=x_a to (x1+range) do
              begin
              PixelColor:=map.Canvas.Pixels[aux2,y2+aux1];
              cr:=ColorToString(PixelColor);
             if cr='clBlack' then
             begin
               count:=count+1;
             end;
          end;
     end;
   end
   else if ((x1>x2) and ((y1<=y2+range) and (y1>=y2-range))) then
   begin
   diff1:=x1-x2;
   if range<=y1 then
     begin
       y_a:=x1-range;
     end
     else
     begin
       y_a:=0;
     end;
       count:=0;
       for aux1:=0 to diff1 do
       begin
          for aux2:=y_a to (Y1+range) do
              begin
              PixelColor:=map.Canvas.Pixels[x1+aux1,aux2];
              cr:=ColorToString(PixelColor);
             if cr='clBlack' then
             begin
               count:=count+1;
             end;
          end;
     end;
   end
   else if ((x1<x2) and ((y1<=y2+range) and (y1>=y2-range))) then
   begin
   diff1:=x2-x1;
   if range<=y1 then
     begin
       y_a:=x1-range;
     end
     else
     begin
       y_a:=0;
     end;
       count:=0;
       for aux1:=0 to diff1 do
       begin
          for aux2:=y_a to (y1+range) do
              begin
              PixelColor:=map.Canvas.Pixels[x2+aux1,aux2];
              cr:=ColorToString(PixelColor);
             if cr='clBlack' then
             begin
               count:=count+1;
             end;
          end;
     end;
    end
    else if (((x1>x2+range) and (x1<x2-range)) and ((y1>y2+range) and (y1<y2-range))) then
    begin
    end;
    check_colour:=count;
end;

procedure Detect_links (p:double;range:integer ;nodelist:a_node;map:TImage);

  var
    l4:integer;
    l5:integer;
    aux4:integer;
    aux5:integer;
    x1:longint;
    y1:longint;
    x2:longint;
    y2:longint;
    n1:integer;
    n2:integer;
    count:integer;
    dist:integer;
    max_x:integer;
    max_y:integer;
    id_nxr:integer;
    id_nxl:integer;
    id_nyr:integer;
    id_nyl:integer;
    nx:longint;
    ny:longint;
    c_count:integer;
  begin
  l4:=length(nodelist);
  if l4>0 then
  begin
  for aux4:=0 to l4-1 do
   begin
   count:=0;
   n1:=nodelist[aux4].id;
   x1:=nodelist[aux4].posnX;
   y1:=nodelist[aux4].posnY;
   id_nxr:=get_node_xr(range,nodelist,x1,y1);
   id_nxl:=get_node_xl(range,nodelist,x1,y1);
   id_nyr:=get_node_yr(range,nodelist,x1,y1);
   id_nyl:=get_node_yl(range,nodelist,x1,y1);
   if id_nxr>0 then
     begin
       n2:=nodelist[id_nxr-1].id;
       x2:=nodelist[id_nxr-1].posnX;
       y2:=nodelist[id_nxr-1].posnY;
       dist:=x2-x1;
       c_count:=check_colour(range,x1,y1,x2,y2,map);
       if c_count>(dist*p) then
       begin
       create_link_between(n1,n2,nodelist);
       end;
     end;
     if id_nxl>0 then
     begin
       n2:=nodelist[id_nxl-1].id;
       x2:=nodelist[id_nxl-1].posnX;
       y2:=nodelist[id_nxl-1].posnY;
       dist:=x1-x2;
       c_count:=check_colour(range,x1,y1,x2,y2,map);
       if c_count>(dist*p) then
       begin
       create_link_between(n1,n2,nodelist);
       end;
     end;
     if id_nyr>0 then
     begin
       n2:=nodelist[id_nyr-1].id;
       x2:=nodelist[id_nyr-1].posnX;
       y2:=nodelist[id_nyr-1].posnY;
       dist:=y2-y1;
       c_count:=check_colour(range,x1,y1,x2,y2,map);
       if c_count>(dist*p) then
       begin
       create_link_between(n1,n2,nodelist);
       end;
     end;
     if id_nyl>0 then
     begin
       n2:=nodelist[id_nyl-1].id;
       x2:=nodelist[id_nyl-1].posnX;
       y2:=nodelist[id_nyl-1].posnY;
       dist:=y1-y2;
       c_count:=check_colour(range,x1,y1,x2,y2,map);
       if c_count>(dist*p) then
       begin
       create_link_between(n1,n2,nodelist);
       end;
     end;
   end;
  end;
  end;

procedure remove_from_nodelist (n1:integer;nodelist:a_node);

  var
    l4:integer;
    l5:integer;
    aux4:integer;
    aux5:integer;
    n_curr:integer;

begin
  l4:=length(nodelist);
  if n1>0 then
  begin
  if l4>0 then
  begin

  for aux4:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux4].id;
     if n_curr=n1 then
     begin
       for aux5:= aux4 to l4-2 do
           begin
           nodelist[aux5]:=nodelist[aux5+1];
           end;
          //SetLength(nodelist, 0);
     end;
  end;
end;
end;
end;

function find_node_id (X:LongInt;Y:LongInt;range:LongInt;nodelist:a_node): integer;

  var
    l4:integer;
    aux4:integer;
    n_curr:integer;
    x_p:LongInt;
    y_p:LongInt;
    flag:integer;
begin
  l4:=length(nodelist);
  flag:=0;
  if l4>0 then
  begin
  for aux4:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux4].id;
     x_p:=nodelist[aux4].posnX;
     y_p:=nodelist[aux4].posnY;
  if ((X<x_p+range) and (X>x_p-range) and (Y<y_p+range) and (Y>y_p-range)) then
     begin
      find_node_id:=n_curr;
      flag:=1;
    end;
 end;
  if flag=0 then
     begin
       find_node_id:=0;
     end;
end;
end;

{ TForm1 }

procedure TForm1.FormCreate(Sender: TObject);
begin
  aux:=0;
  Canvas.Pen.Width:=5;
  Canvas.Pen.Color:=clRed;
end;

procedure TForm1.FormPaint(Sender: TObject);
begin
  Canvas.Pen.Width:=5;
  Canvas.Pen.Color:=clRed;
  l4:=length(intersection_nodesXY);
    if l4>0 then
    begin
    for aux4:=0 to l4-1 do
    begin
     pointX:=intersection_nodesXY[aux4].posnX+280;
     pointY:=intersection_nodesXY[aux4].posnY+29;
     x_temp:=pointX+1;
     y_temp:=pointY+1;
     Canvas.Rectangle (pointX-1,pointY-1,x_temp,y_temp);
    end;
    end;


end;

procedure TForm1.FormShow(Sender: TObject);
begin
    Canvas.Pen.Width:=5;
  Canvas.Pen.Color:=clRed;
  l4:=length(intersection_nodesXY);
    if l4>0 then
    begin
    for aux4:=0 to l4-1 do
    begin
     pointX:=intersection_nodesXY[aux4].posnX+280;
     pointY:=intersection_nodesXY[aux4].posnY+29;
     x_temp:=pointX+1;
     y_temp:=pointY+1;
     Canvas.Rectangle (pointX-1,pointY-1,x_temp,y_temp);
    end;
    end;
end;

procedure TForm1.Label3Click(Sender: TObject);
begin

end;

procedure TForm1.mapClick(Sender: TObject);
begin
  //Processes the placement and removal of nodes on the map
  if ((form1.vel_nom11.text='') or (form1.width1.text='') or (form1.heigth.text='')) then
  Begin
  Label5.Caption:='All fields Must be filled before creating any nodes!!';
  end
  else
  begin
  o_w:=StrtoFloat(form1.width1.text);
  o_h:=StrtoFloat(form1.heigth.text);
  vel_nom:=StrtoFloat(vel_nom11.text);
  if Add_toggle.Checked = True then
  begin
    pointX:=Mouse.CursorPos.X;
    pointY:=Mouse.CursorPos.y;
    pointg.x:=pointX;
    pointg.y:=pointY;
    pointl:=Form1.ScreenToClient(pointg);

    PixelColor:=map.Canvas.Pixels[pointl.x-280,pointl.y-29];
    Label6.Caption:=ColorToString(Pixelcolor);


    node_temp.Init;
    node_temp.id:=aux+1;
    node_temp.posnX:=pointl.x-280;
    node_temp.posnY:=pointl.y-29;
    node_temp.posrealX:=(pointl.x-280)*(o_w/913);
    node_temp.posrealY:=(pointl.y-29)*(o_h/617);
    l:=length(intersection_nodesXY);
    SetLength(intersection_nodesXY, l+1);
    intersection_nodesXY[l]:=node_temp;
    x_temp:=pointl.x-279;
    y_temp:=pointl.y-28;
    Canvas.Rectangle (pointl.x-281,pointl.y-30,x_temp,y_temp);
    Label3.Caption:=FloatToStr(intersection_nodesXY[l].posrealX);
    Label4.Caption:=FloatToStr(intersection_nodesXY[l].posrealY);
    aux:= aux+1;
  end
  else if ToggleBox1.Checked=True then
  begin
    pointX:=Mouse.CursorPos.X;
    pointY:=Mouse.CursorPos.y;
    pointg.x:=pointX;
    pointg.y:=pointY;
    pointl:=Form1.ScreenToClient(pointg);
    pointX:=pointl.x-280;
    pointY:=pointl.y-29;

    id_r:=find_node_id(pointX,pointY,5,intersection_nodesXY);
    remove_from_nodelist(id_r,intersection_nodesXY);

    if id_r>0 then
    begin
    l4:=length(intersection_nodesXY);
    setlength(intersection_nodesXY,l4-1);
    end;
    Invalidate;

  end;
end;
end;

procedure TForm1.mapPaint(Sender: TObject);
begin
  Canvas:=map.Canvas;
  Canvas.Pen.Width:=5;
  Canvas.Pen.Color:=clRed;
  l4:=length(intersection_nodesXY);
    if l4>0 then
    begin
    for aux4:=0 to l4-1 do
    begin
     pointX:=intersection_nodesXY[aux4].posnX;
     pointY:=intersection_nodesXY[aux4].posnY;
     x_temp:=pointX+1;
     y_temp:=pointY+1;
     Canvas.Rectangle (pointX-1,pointY-1,x_temp,y_temp);
    end;
    end;
end;

procedure TForm1.RadioGroup1Click(Sender: TObject);
begin

end;

procedure TForm1.scaleChange(Sender: TObject);
begin

end;

procedure TForm1.ToggleBox1Change(Sender: TObject);
begin

end;




procedure TForm1.Button1Click(Sender: TObject);
begin
  //Label3.Caption:=scale.text;
   o_w:=StrtoFloat(form1.width1.text);
   o_h:=StrtoFloat(form1.heigth.text);
   vel_nom:=StrtoFloat(vel_nom11.text);
   if (RadioButton1.Checked= true) then
   begin
   p:=1;
   Detect_links(p,5,intersection_nodesXY,form1.map);
    //create_link_between(1,2,intersection_nodesXY);
   //dist:=intersection_nodesXY[1].posnX-intersection_nodesXY[0].posnX;
   // c_count:=check_colour(5,intersection_nodesXY[0].posnX,intersection_nodesXY[0].posnY,intersection_nodesXY[1].posnX,intersection_nodesXY[1].posnY,map);
   //Label7.Caption:=floatToStr(dist*p*5);
   //Label8.Caption:=intToStr(c_count);
    end;
   Form2.Show;
   Form1.hide;
end;

procedure TForm1.Add_toggleClick(Sender: TObject);
begin

end;



procedure TForm1.Button2Click(Sender: TObject);
begin
  if not OpenPictureDialog1.Execute then exit;
            OpenPictureDialog1.Options:= OpenPictureDialog1.Options+[ofFileMustExist];
            map.Picture.LoadFromFile(OpenPictureDialog1.Filename);
end;

procedure TForm1.Button3Click(Sender: TObject);
begin
   // For vCntr:= 0 to Length(intersection_nodesXY) do
   //begin
   //    node_temp:=intersection_nodesXY[vCntr];
   //    intersection_nodesXY[vCntr] := null;
   //
   // end;
    SetLength(intersection_nodesXY, 0);
    Invalidate;
end;


end.

