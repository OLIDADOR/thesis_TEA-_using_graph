unit Unit2;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls, Grids,
  ExtCtrls, GLScene, GLGraph, GLFullScreenViewer, GLCadencer, GLObjects,
  GLLCLViewer, Dom, XmlRead, XMLWrite, Math, Types, GLBaseClasses,character,TEAstar;


const

  // Cell States
  VIRGIN = 0;
  OBSTACLEWALL = 1;
  CLOSED = 2;
  OPENED = 3;
  OBSTACLEROBOT = 4;

  NUM_LAYERS = 160;
  NUMBER_ROBOTS = 4;
  MAX_EXCHANGES = 10;
  MAX_ITERATIONS = 10000;
  MAX_SUBMISSIONS = 4;
  //Max_nodes=form1.graphsize;
  //AStarHeapArraySize = form1.graphsize*NUM_LAYERS;

  COST1 = 0.5;
  COST2 = 0.5;
  COST3 = 0.707;

type

  { TForm2 }

  TForm2 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    GLCadencer1: TGLCadencer;
    GLCamera3: TGLCamera;
    GLCube3: TGLCube;
    GLDummyCube3: TGLDummyCube;
    GLLightSource3: TGLLightSource;
    GLPlane1: TGLPlane;
    GLScene3: TGLScene;
    GLSceneViewer1: TGLSceneViewer;
    GLSceneViewer2: TGLSceneViewer;
    Label1: TLabel;
    Label2: TLabel;
    LabeledEdit1: TLabeledEdit;
    LabeledEdit2: TLabeledEdit;
    StringGrid1: TStringGrid;
    StringGrid2: TStringGrid;
    procedure Button1Click(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure FormPaint(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure GLCadencer1Progress(Sender: TObject; const deltaTime,
      newTime: Double);
    procedure GLSceneViewer1MouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure GLSceneViewer1MouseMove(Sender: TObject; Shift: TShiftState; X,
      Y: Integer);
    procedure GLSceneViewer1MouseWheel(Sender: TObject; Shift: TShiftState;
      WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
    procedure ToggleBox1Change(Sender: TObject);
  private

  public

  end;



var
  Form2: TForm2;
  mx, my, mx2, my2: integer;
  l1,l2:integer;
  aux1, aux2:integer;
  X:double;
  Y:double;
  i_curr:integer;
  count:integer;
  r_ID:integer;
  r_id_curr:integer;
  ws_ID:integer;
  rc1,rc2:integer;
  i_t:integer;
  i_node:integer;
implementation
      uses
   unit1,main,controlo;

{$R *.lfm}

function get_line_Dir (x1:Double;y1:Double;x2:double;y2:Double):integer;

begin
   if (x1=x2) and (y1<>y2) then
      begin
       get_line_Dir:=1;
      end
    else if (x1<>x2) and (y1=y2) then
      begin
       get_line_Dir:=2;
      end
    else
    begin
    get_line_Dir:=0;
    end;
end;



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



procedure Print_map_in_GLS(nodelist:a_node; width_line:integer; GLScene: TGLScene; base:TGLDummyCube ; scale:integer);
      var
       newline: TGLLines;
       id_n:integer;
       l1:integer;
       l2:integer;
       l3:integer;
       n_curr:integer;
       ntl:integer;
       x1:Double;
       y1:Double;
       x2:Double;
       y2:Double;
       c:integer;
       aux4:integer;
       aux5:integer;
       aux6:integer;
       dist:real;
       declive:real;
       angle:real;
       l_id:integer;
       f_done:integer;
       count:integer;
       res:integer;
       l_done:array of integer;
       begin
         l1:=length(nodelist);
       for aux4:=0 to l1-1 do
        begin
         x1:=nodelist[aux4].pos_X*scale;
         y1:=nodelist[aux4].pos_y*scale;
         l2:=length(nodelist[aux4].links);
         for aux5:=0 to l2-1 do
          begin
           ntl:=nodelist[aux4].links[aux5].node_to_link;
           dist:=nodelist[aux4].links[aux5].distance *scale;
           l_id:=nodelist[aux4].links[aux5].id_l;
           f_done:=check_array(l_done,l_id);
           if f_done=0 then
           begin
            for aux6:=0 to l1-1 do
             begin
             n_curr:=nodelist[aux6].id;
             if n_curr=ntl then
                begin
                  x2:=nodelist[aux6].pos_X*scale;
                  y2:=nodelist[aux6].pos_y*scale;

                  end;
                  end;
                  res:=get_line_Dir(x1,y1,x2,y2);
                  newline:=TGLLines.CreateAsChild(GLScene.Objects);
                  newline.LineWidth:=width_line;
                  if ((res=1) and (y2>y1)) then
                  begin
                  newline.AddNode(x1-1.5*scale,y1-1.1*scale-(width_line/4),0);
                  newline.AddNode(x2-1.5*scale,y2-1.1*scale+(width_line/4),0);
                  end
                  else if ((res=1) and (y2<y1)) then
                  begin
                  newline.AddNode(x1-1.5*scale,y1-1.1*scale+(width_line/4),0);
                  newline.AddNode(x2-1.5*scale,y2-1.1*scale-(width_line/4),0);
                  end
                  else if ((res=2) and (x2>x1)) then
                  begin
                  newline.AddNode(x1-1.5*scale-width_line/4,y1-1.1*scale,0);
                  newline.AddNode(x2-1.5*scale+width_line/4,y2-1.1*scale,0);
                  end
                  else if ((res=2) and (x2<x1)) then
                  begin
                  newline.AddNode(x1-1.5*scale+width_line/4,y1-1.1*scale,0);
                  newline.AddNode(x2-1.5*scale-width_line/4,y2-1.1*scale,0);
                  end
                  else
                  begin
                  newline.AddNode(x1-1.5*scale,y1-1.1*scale,0);
                  newline.AddNode(x2-1.5*scale,y2-1.1*scale,0);
                  end;
                  //GLScene.Objects.addchild(newline);
                  l3:=length(l_done);
                  setlength(l_done,l3+1);
                  l_done[l3]:=l_id;
                  end;
           end;
          end;
       end;





procedure print_robot_position_GLS(robotlist:r_node; scale:integer; GLScene: TGLScene; base:TGLDummyCube; r_w:double; r_h:double);
var
l1:integer;
l2:integer;
aux1:integer;
aux2:integer;
x:double;
y:double;
newcube: TGLCube;
angle:double;
 begin
    l1:=length(robotlist);
    if l1>0 then
    begin
    for aux1:=0 to l1-1 do
      begin
         x:=robotlist[aux1].pos_X*scale;
         y:=robotlist[aux1].pos_Y*scale;
         newcube:=TGLCube.CreateAsChild(GLScene.Objects);
         newcube.CubeHeight:=r_h;
         newcube.CubeWidth:=r_w;
         newcube.CubeDepth:=1;
         newcube.Position.X:=x-1.5*scale;
         newcube.Position.y:=y-1.1*scale;
         newcube.Position.z:=1;
         angle:=gradtorad(robotlist[aux1].Direction);
         //newcube.Direction.X:=cos(angle);
         //newcube.Direction.Y:=sin(angle);
         //newcube.Direction.Z:=1;
         //colour.
        newcube.Material.FrontProperties.Ambient.RandomColor;
      end;
 end;
 end;


procedure add_mission(var agv:Robot_Pos_info; nid:integer);
var
l1:integer;
begin
     if agv.NumberSubMissions>0 then
     begin
         agv.SubMissions[agv.NumberSubMissions-1]:=nid;
         agv.NumberSubMissions:=agv.NumberSubMissions+1;
     end
     else
     begin
       agv.target_node:=nid;
       agv.SubMissions[0]:=nid;
       agv.ActualSubMission:=1;
       agv.NumberSubMissions:=1;
       agv.CounterSubMissions:=1;
     end;

end;
 { TForm2 }


 procedure TForm2.GLCadencer1Progress(Sender: TObject; const deltaTime,
  newTime: Double);
begin
   if ((mx <> mx2) or (my <> my2)) then
  begin
    GLCamera3.MoveAroundTarget(my - my2, mx - mx2);
    mx := mx2;
    my := my2;
  end;
end;

procedure TForm2.FormShow(Sender: TObject);
begin
 Print_map_in_GLS(form1.full_nodelist,10,GLScene3,GLDummyCube3,200);
 Print_robot_position_GLS(form1.robots,200,GLScene3,GLDummyCube3,25,25);
 l1:=length(form1.robots);
 label2.Caption:=inttostr(l1);
end;

procedure TForm2.Button1Click(Sender: TObject);
begin
  r_ID:=strtoint(labelededit1.Text);
  ws_ID:=strtoint(labelededit2.Text);
  l1:=length(form1.robots);
  for aux1:=0 to l1-1 do
  begin
    r_id_curr:=form1.robots[aux1].id_robot;
    if r_id_curr=r_ID then
    begin
       add_mission(form1.robots[aux1],form1.ws[ws_ID-1]);
       //form1.robots[aux1].target_node:=form1.ws[ws_ID-1];
    end;
  end;
end;

procedure TForm2.Button2Click(Sender: TObject);
begin
  Application.CreateForm(TFMain, FMain);
  Application.CreateForm(TFControlo, FControlo);
  FMain.Show;
  FControlo.Show;
end;

procedure TForm2.FormPaint(Sender: TObject);
begin
  rc1:=StringGrid1.RowCount;
  for aux1:=1 to rc1-1 do
     begin
     StringGrid1.DeleteRow(1);
     end;
  rc2:=StringGrid2.RowCount;
  for aux1:=1 to rc2-1 do
     begin
     StringGrid2.DeleteRow(1);
     end;
  l1:=length(form1.full_nodelist);
  count:=1;
  for aux1:=0 to l1-1 do
  begin
  i_curr:=form1.full_nodelist[aux1].id;
  x:=form1.full_nodelist[aux1].pos_X;
  y:=form1.full_nodelist[aux1].pos_y;
  if check_array(form1.ws,i_curr)=1 then
  begin
       StringGrid1.InsertRowWithValues(1,[inttostr(count),inttostr(i_curr) , floattostr(x), floattostr(Y)]);
       count:=count+1;
  end;
  end;
  l1:=length(form1.robots);
  for aux1:=0 to l1-1 do
  begin
     i_curr:=form1.robots[aux1].id_robot;
     i_t:=form1.robots[aux1].target_node;
     i_node:= form1.robots[aux1].inicial_node;
     l2:=length(form1.full_nodelist);
     for aux2:=0 to l2-1 do
     begin
     if form1.full_nodelist[aux2].id=i_node  then
     begin
     X:=form1.full_nodelist[aux2].pos_X;
     Y:=form1.full_nodelist[aux2].pos_Y;
     end;
     end;
     if check_array(form1.ws,i_t)=1 then
     begin
     StringGrid2.InsertRowWithValues(1,[inttostr(i_node), floattostr(X),floattostr(Y), inttostr(i_t)]);
     end
     else
     begin
       StringGrid2.InsertRowWithValues(1,[inttostr(i_curr), floattostr(X),floattostr(Y)]);
     end;
  end;
end;

procedure TForm2.GLSceneViewer1MouseDown(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin
  mx := X;
  my := Y;
  mx2 := X;
  my2 := Y;
end;

procedure TForm2.GLSceneViewer1MouseMove(Sender: TObject; Shift: TShiftState;
  X, Y: Integer);
begin
   if ssLeft in Shift then
  begin
    mx2 := X;
    my2 := Y;
  end;
end;

procedure TForm2.GLSceneViewer1MouseWheel(Sender: TObject; Shift: TShiftState;
  WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
begin
   GLCamera3.AdjustDistanceToTarget(Power(1.025, WheelDelta / 300));
end;

procedure TForm2.ToggleBox1Change(Sender: TObject);
begin

end;



end.

