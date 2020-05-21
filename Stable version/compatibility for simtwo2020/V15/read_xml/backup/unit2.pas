unit Unit2;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls, Grids,
  ExtCtrls, GLScene, GLGraph, GLFullScreenViewer, GLCadencer, GLObjects,
  GLLCLViewer, Dom, XmlRead, XMLWrite, Math, Types, GLBaseClasses,character,TEAstar;



type

  { TForm2 }

  TForm2 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    Button3: TButton;
    Button4: TButton;
    GLCadencer1: TGLCadencer;
    GLCadencer2: TGLCadencer;
    GLCamera3: TGLCamera;
    GLCube3: TGLCube;
    GLDummyCube3: TGLDummyCube;
    GLLightSource3: TGLLightSource;
    GLPlane1: TGLPlane;
    GLScene3: TGLScene;
    GLSceneViewer1: TGLSceneViewer;
    Label1: TLabel;
    Label2: TLabel;
    LabeledEdit1: TLabeledEdit;
    LabeledEdit2: TLabeledEdit;
    LabeledEdit3: TLabeledEdit;
    LabeledEdit4: TLabeledEdit;
    LabeledEdit5: TLabeledEdit;
    LabeledEdit6: TLabeledEdit;
    PaintBox1: TPaintBox;
    StringGrid1: TStringGrid;
    StringGrid2: TStringGrid;
    Timer1: TTimer;
    ToggleBox1: TToggleBox;
    procedure Button1Click(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure Button3Click(Sender: TObject);
    procedure Button4Click(Sender: TObject);
    procedure FormPaint(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure GLCadencer1Progress(Sender: TObject; const deltaTime,
      newTime: Double);
    procedure GLCadencer2Progress(Sender: TObject; const deltaTime,
      newTime: Double);
    procedure GLSceneViewer1MouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure GLSceneViewer1MouseMove(Sender: TObject; Shift: TShiftState; X,
      Y: Integer);
    procedure GLSceneViewer1MouseWheel(Sender: TObject; Shift: TShiftState;
      WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
    procedure PaintBox1Paint(Sender: TObject);
    procedure Timer1Timer(Sender: TObject);
  private

  public
  coms_flaws:integer;
  X_c_max,X_c_min,Y_c_max,Y_c_min: array of Double;
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
  printedmap:integer;
  u_ws:array of integer;
implementation
      uses
   unit1,main,controlo,unit3;

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

function checkifflawnode(id:integer; flaws:controlo.coms_flaw_location):integer;
var
  aux1,aux2,aux3,at,s1,s2,s3:integer;
begin
   s1:=length(flaws.detected_nodes);
   at:=0;
   if s1>0 then begin
   for aux1:=0 to s1-1 do begin
      s2:=length(flaws.detected_nodes[aux1]);
      if s2>0 then begin
      for aux2:=0 to s2-1 do begin
           if flaws.detected_nodes[aux1][aux2]=id then begin
            at:=1;
           end;
         end;
   end;
   end;
end;
   checkifflawnode:=at;
end;

function get_ws_ID(id:integer):integer;
var
  size1,aux1:integer;
begin
   size1:=length(form3.wos);
   for aux1:=0 to size1-1 do
   begin
   if id=form3.wos[aux1].ws_id then begin
     get_ws_ID:=aux1;
   end;
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

  function getXcoord(n1:integer):Double;
var
l1:integer;
aux1,n_id:integer;
c:double;
begin
 l1:=length(form1.full_nodelist);
   c:=999999999;
   for aux1:=0 to l1-1 do
   begin
     n_id:=form1.full_nodelist[aux1].id;
     if n_id=n1 then
     begin
      c:=form1.full_nodelist[aux1].pos_X;
     end;
   end;
   getXcoord:=c;
end;

function getYcoord(n1:integer):Double;
var
l1:integer;
aux1,n_id:integer;
c:double;
begin
 l1:=length(form1.full_nodelist);
   c:=999999999;
   for aux1:=0 to l1-1 do
   begin
     n_id:=form1.full_nodelist[aux1].id;
     if n_id=n1 then
     begin
      c:=form1.full_nodelist[aux1].pos_Y;

     end;
   end;
   getYcoord:=c;
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
         form1.robots[aux1].cube:=newcube;
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
         agv.NumberSubMissions:=agv.NumberSubMissions+1;
         agv.SubMissions[agv.NumberSubMissions-1]:=nid;
     end
     else
     begin
       agv.target_node:=nid;
       agv.SubMissions[0]:=nid;
       agv.ActualSubMission:=1;
       agv.NumberSubMissions:=1;
       agv.CounterSubMissions:=1;
       agv.TotalSubMissions:=1;
     end;

end;
function checkarray(a:array of integer; i:integer):integer;
 var
 le1,aux1,a_id,r:integer;
begin
  le1:=length(a);
  r:=0;
  if le1>0 then
  begin
  for aux1:=0 to le1-1 do
    begin
      a_id:=a[aux1];
      if a_id=i then
      begin
        r:=1;
      end;
    end;
  end;
  checkarray:=r;
end;

function getclosesestrestplace(nodelist:a_node;ws:w_node;id:integer):integer;

var
    le4,le5,le6:integer;
    aux4,aux5:integer;
    n_curr,w_curr:integer;
    x_p:Double;
    y_p:Double;
    x:Double;
    y:Double;
    id_min:integer;
    diff1:Double;
    diff2:Double;
    Difft:Double;
    diff_min:Double;
begin
  le4:=length(ws);
  le5:=length(nodelist);
  le6:=length(u_ws);
  diff_min:=10;
  id_min:=0;
  if le5>0 then
  begin
  for aux5:=0 to le5-1 do
  begin
    n_curr:=nodelist[aux5].id;
    if n_curr=id then
    begin
    x:=nodelist[aux5].pos_X;
    y:=nodelist[aux5].pos_y;
    end;
  end;
  if le4>0 then
  begin
  for aux4:=0 to le4-1 do
  begin
     w_curr:=ws[aux4].node_id;
     x_p:=ws[aux4].pos_X;
     y_p:=ws[aux4].pos_Y;
     diff1:=abs(x_p-x);
     diff2:=abs(y_p-y);
     Difft:=diff1+diff2;
  if ((diff_min>Difft) and (ws[aux4].isactive<>1) and (checkarray(u_ws,ws[aux4].node_id)<>1) and (le6>0)) then
     begin
      diff_min:=Difft;
      id_min:=w_curr;
  end else if ((diff_min>Difft) and (ws[aux4].isactive<>1) and (le6=0)) then
     begin
      diff_min:=Difft;
      id_min:=w_curr;
      end;
 end;
  getclosesestrestplace:=id_min;
end;
end;
end;



procedure Setrobotsrestspot(nodelist:a_node; robotlist:r_node;ws:w_node);
var
le1,le2,aux1,aux5,final_mission_ind,final_mission_id,idr:integer;
begin
    le1:=length(robotlist);
    le2:=length(u_ws);
    if le1>0 then
    begin
    for aux1:=0 to le1-1 do
      begin
        final_mission_ind:=robotlist[aux1].NumberSubMissions;
        final_mission_id:=robotlist[aux1].SubMissions[final_mission_ind-1];
        idr:=getclosesestrestplace(nodelist,ws,final_mission_id);
        if le2=0 then begin
        setlength(u_ws,le1);
        for aux5:=0 to le2-1 do
        begin
        u_ws[aux5]:=0;
        end;
        u_ws[aux1]:=idr;
        end else begin
         u_ws[aux1]:=idr;
        end;
        add_mission(robotlist[aux1], idr);
      end;
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

procedure TForm2.GLCadencer2Progress(Sender: TObject; const deltaTime,
  newTime: Double);
var
l1,aux1:integer;
begin
   if printedmap=1 then
   begin
     l1:=length(form1.robots);
     for aux1:=0 to l1-1 do
     begin
     form1.robots[aux1].cube.Position.X:=form1.robots[aux1].pos_X*200-1.5*200;
     form1.robots[aux1].cube.Position.Y:=form1.robots[aux1].pos_y*200-1.1*200;
     end;
  end;
  if ToggleBox1.Checked=True then
    begin
     coms_flaws:=1;
    end else
    begin
     coms_flaws:=0;
    end;
end;

procedure TForm2.FormShow(Sender: TObject);
begin
 Print_map_in_GLS(form1.full_nodelist,10,GLScene3,GLDummyCube3,200);
 Print_robot_position_GLS(form1.robots,200,GLScene3,GLDummyCube3,25,25);
 printedmap:=1;
 l1:=length(form1.robots);
 label2.Caption:=inttostr(l1);
end;

procedure TForm2.Button1Click(Sender: TObject);
var
auxID:integer;
begin
  r_ID:=strtoint(labelededit1.Text);
  ws_ID:=strtoint(labelededit2.Text);
  l1:=length(form1.robots);
  for aux1:=0 to l1-1 do
  begin
    r_id_curr:=form1.robots[aux1].id_robot;
    if r_id_curr=r_ID then
    begin
       auxID:=get_ws_ID(ws_ID);
       add_mission(form1.robots[aux1],form3.wos[auxID].node_id);
       //form1.robots[aux1].target_node:=form1.ws[ws_ID-1];
    end;
  end;
end;

procedure TForm2.Button2Click(Sender: TObject);
begin
  Setrobotsrestspot(form1.full_nodelist,form1.robots,form3.wos);
  Application.CreateForm(TFMain, FMain);
  Application.CreateForm(TFControlo, FControlo);
  FMain.Show;
  FControlo.Show;
end;

procedure TForm2.Button3Click(Sender: TObject);
var
s1,s2,s3,s4:integer;
begin
 If coms_flaws=1 then begin
  if LabeledEdit3.text<>'' then begin
     s1:=length(X_c_max);
     setlength(X_c_max,s1+1);
     X_c_max[s1]:=strtofloat(LabeledEdit3.text);
     end;
     if LabeledEdit4.text<>'' then begin
     s2:=length(X_c_min);
     setlength(X_c_min,s2+1);
     X_c_min[s2]:=strtofloat(LabeledEdit4.text);
     end;
     if LabeledEdit5.text<>'' then begin
     s3:=length(Y_c_max);
     setlength(Y_c_max,s3+1);
     Y_c_max[s3]:=strtofloat(LabeledEdit5.text);
     end;
     if LabeledEdit6.text<>'' then begin
     s4:=length(Y_c_min);
     setlength(Y_c_min,s4+1);
     Y_c_min:=strtofloat(LabeledEdit6.text);
     end;
 end;
end;

procedure TForm2.Button4Click(Sender: TObject);
var
Doc: TXMLDocument;                                  // variable to document
RootNode, parentNode, nofilho: TDOMNode;                    // variable to nodes
l1,l2,l3,aux1,aux2:integer;
begin
  // Create a document
 Doc := TXMLDocument.Create;

 // Create a root node
 RootNode := Doc.CreateElement('Mission');
 Doc.Appendchild(RootNode);      // save root node

 l1:=length(form1.robots);
 for aux1:=0 to l1-1 do
 begin
 // Create a defines node
  RootNode:= Doc.DocumentElement;
  parentNode := Doc.CreateElement('Robot');
  TDOMElement(parentNode).SetAttribute('id',inttostr(form1.robots[aux1].id_robot));     // create atributes
  RootNode.Appendchild(parentNode);

  parentNode := Doc.CreateElement('priority');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].InitialIdPriority));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('x');               // create a child node
  nofilho := Doc.CreateTextNode(FloatToStr(form1.robots[aux1].pos_X));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('y');               // create a child node
  nofilho := Doc.CreateTextNode(FloatToStr(form1.robots[aux1].pos_y));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('Direction');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].Direction));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('node');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].inicial_node));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('steps');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].inicial_step));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('ActualSubMission');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].ActualSubMission));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('CounterSubMissions');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].CounterSubMissions));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('TotalSubMissions');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].TotalSubMissions));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('NumberSubMissions');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].NumberSubMissions));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('target_node');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].target_node));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('target_node_step');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].target_node_step));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  parentNode := Doc.CreateElement('SubMissions');               // create a child node
  nofilho := Doc.CreateTextNode(inttostr(length(form1.robots[aux1].SubMissions)));               // insert a value to node
  parentNode.Appendchild(nofilho);                         // save node
  RootNode.ChildNodes.Item[aux1].AppendChild(parentNode);     // insert a childnode in respective pai

  l2:=length(form1.robots[aux1].SubMissions);
  for aux2:=0 to l2-1 do
  begin
      parentNode := Doc.CreateElement('SubMission'+inttostr(form1.robots[aux1].id_robot));               // create a child node
      nofilho := Doc.CreateTextNode(inttostr(form1.robots[aux1].SubMissions[aux2]));               // insert a value to node
      parentNode.Appendchild(nofilho);                         // save node
      RootNode.ChildNodes.Item[aux1].ChildNodes.Item[11].AppendChild(parentNode);     // insert a childnode in respective pai
  end;
end;
    l3:=length(form3.wos);
    for aux1:=0 to l3-1 do
    begin
     parentNode := Doc.CreateElement('Workstation');
     TDOMElement(parentNode).SetAttribute('id',inttostr(form3.wos[aux1].id));     // create atributes
     RootNode.Appendchild(parentNode);

     parentNode := Doc.CreateElement('node_id');               // create a child node
     nofilho := Doc.CreateTextNode(inttostr(form3.wos[aux1].node_id));               // insert a value to node
     parentNode.Appendchild(nofilho);                         // save node
     RootNode.ChildNodes.Item[l1+aux1].AppendChild(parentNode);     // insert a childnode in respective pai

     parentNode := Doc.CreateElement('ws_id');               // create a child node
     nofilho := Doc.CreateTextNode(inttostr(form3.wos[aux1].ws_id));               // insert a value to node
     parentNode.Appendchild(nofilho);                         // save node
     RootNode.ChildNodes.Item[l1+aux1].AppendChild(parentNode);     // insert a childnode in respective pai

     parentNode := Doc.CreateElement('isactive');               // create a child node
     nofilho := Doc.CreateTextNode(inttostr(form3.wos[aux1].isactive));               // insert a value to node
     parentNode.Appendchild(nofilho);                         // save node
     RootNode.ChildNodes.Item[l1+aux1].AppendChild(parentNode);     // insert a childnode in respective pai

     parentNode := Doc.CreateElement('X');               // create a child node
     nofilho := Doc.CreateTextNode(Floattostr(form3.wos[aux1].pos_X));               // insert a value to node
     parentNode.Appendchild(nofilho);                         // save node
     RootNode.ChildNodes.Item[l1+aux1].AppendChild(parentNode);     // insert a childnode in respective pai

     parentNode := Doc.CreateElement('Y');               // create a child node
     nofilho := Doc.CreateTextNode(Floattostr(form3.wos[aux1].pos_y));               // insert a value to node
     parentNode.Appendchild(nofilho);                         // save node
     RootNode.ChildNodes.Item[l1+aux1].AppendChild(parentNode);     // insert a childnode in respective pai
    end;
     writeXMLFile(Doc, 'Mission.xml');                     // write to XML
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
  l1:=length(form3.wos);
  count:=1;
  for aux1:=0 to l1-1 do
  begin
  i_curr:=form3.wos[aux1].id;
  x:=form3.wos[aux1].pos_X;
  y:=form3.wos[aux1].pos_y;
  if form3.wos[aux1].isactive=1 then
  begin
       StringGrid1.InsertRowWithValues(1,[inttostr(count),inttostr(i_curr) , floattostr(x), floattostr(Y)]);
       form3.wos[aux1].ws_id:=count;
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


procedure TForm2.PaintBox1Paint(Sender: TObject);
var
  l1,l2,tt,ttt,l3,id_l,aux1,aux2,aux3,id_i:integer;
  X_print,Y_print,X_print1,Y_print1:Longint;
  aux_flaw:controlo.coms_flaw_location;
begin
  aux_flaw:=controlo.flaw_location;
  tt:=length(aux_flaw.detected_nodes);
  if tt>0 then begin
  ttt:=length(aux_flaw.detected_nodes[0]);
  if ttt>2 then begin
  ttt:=0;
  end;
  end;
  Canvas := form2.PaintBox1.Canvas;
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
  //Print Nodes
    Canvas.Pen.Width:=8;
    Canvas.Pen.Color:=clRed;
   l3:=length(form1.full_nodelist);
   if l3>0 then begin
   for aux1:=0 to l3-1 do
       begin
          id_i:=form1.full_nodelist[aux1].id;
          X_print:=round(form1.full_nodelist[aux1].pos_X*200);
          Y_print:=round(form1.full_nodelist[aux1].pos_y*200);
          if checkifflawnode(id_i,aux_flaw)=1 then
          begin
             Canvas.Pen.Color:=clRed;
          end
          else if checkifflawnode(id_i,aux_flaw)=0 then
          begin
             Canvas.Pen.Color:=clblue;
          end;
          Canvas.Rectangle (X_print-1,Y_print-1,X_print+1,Y_print+1);
       end;
   end;

end;

procedure TForm2.Timer1Timer(Sender: TObject);
begin
  invalidate;
end;


end.

