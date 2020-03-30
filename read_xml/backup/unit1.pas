unit Unit1;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls, Grids,
  ExtCtrls, GLScene, GLGraph, GLFullScreenViewer, GLCadencer, GLObjects,
  GLLCLViewer, Dom, XmlRead, XMLWrite, Math, Types, GLBaseClasses;

type
    link_full = object
   private
     {private declarations}
   public
     {public declarations}
     var
     id_l:integer;
     node_to_link:integer;
     distance:real;
   end;
  node_full = object
   private
     {private declarations}
   public
     {public declarations}
     var
     id:integer;
     pos_X:Double;
     pos_Y:Double;
     defined:integer;
     number_of_links:integer;
     links:array of link_full;
   end;

   Robot_Pos_info = object
   private
     {private declarations}
   public
     {public declarations}
     var
     id_robot:integer;
     current_nodes:array of integer;
     pos_X:Double;
     pos_Y:Double;
   end;

       r_node=array of Robot_Pos_info;

      a_node=array of node_full;



  { TForm1 }

  TForm1 = class(TForm)
    Button1: TButton;
    GLCadencer1: TGLCadencer;
    GLCamera1: TGLCamera;
    GLCube1: TGLCube;
    GLCube2: TGLCube;
    GLDummyCube3: TGLDummyCube;
    GLLightSource1: TGLLightSource;
    GLScene2: TGLScene;
    GLSceneViewer1: TGLSceneViewer;
    LabeledEdit2: TLabeledEdit;
    LabeledEdit3: TLabeledEdit;
    StringGrid1: TStringGrid;
    StringGrid2: TStringGrid;
    procedure Button1Click(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure GLCadencer1Progress(Sender: TObject; const deltaTime,
      newTime: Double);
    procedure GLSceneViewer1MouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure GLSceneViewer1MouseMove(Sender: TObject; Shift: TShiftState; X,
      Y: Integer);
    procedure GLSceneViewer1MouseWheel(Sender: TObject; Shift: TShiftState;
      WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
  private

  public

  end;

var
  Form1: TForm1;
  full_nodelist:array of node_full;
  l1:integer;
  l4:integer;
  aux1:integer;
  i_curr:integer;
  x:double;
  y:double;
  Nodes: TDOMNodeList;
  Node: TDOMNode;
  id:TDOMNode;
  Doc: TXMLDocument;
  id_l:integer;
  aux2:integer;
  l2:integer;
  ntl:integer;
  dist:double;
  x_r:double;
  y_r:double;
  newline1: TGLLines;
    mx, my, mx2, my2: integer;
    robots:array of Robot_Pos_info;
    max_id:integer;
    id_r:integer;
implementation

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

function get_closest_node_id (nodelist:a_node; x:Double; y:Double; scale:integer):integer;

var
    l4:integer;
    aux4:integer;
    n_curr:integer;
    x_p:Double;
    y_p:Double;
    id_min:integer;
    diff1:Double;
    diff2:Double;
    Difft:Double;
    diff_min:Double;
begin
  l4:=length(nodelist);
  diff_min:=10*scale;
  id_min:=0;
  if l4>0 then
  begin
  for aux4:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux4].id;
     x_p:=nodelist[aux4].pos_X*scale;
     y_p:=nodelist[aux4].pos_Y*scale;
     diff1:=abs(x_p-x*scale);
     diff2:=abs(y_p-y*scale);
     Difft:=diff1+diff2;
  if diff_min>Difft then
     begin
      diff_min:=Difft;
      id_min:=n_curr;
    end;
 end;
  get_closest_node_id:=id_min;
end;
end;

function get_max_robotid(robotlist:r_node):integer;
var
l4:integer;
aux4:integer;
i_max:integer;
i_curr:integer;
begin
 l4:=length(robotlist);
 i_max:=0;
 for aux4:=0 to l4-1 do
 begin
    i_curr:=robotlist[aux4].id_robot;
    if i_curr>i_max then
       begin
       i_max:=i_curr;
       end;
 end;
  get_max_robotid:=i_max;
end;

procedure update_robot_inicial_position(r_id:integer; id:integer; robotlist:r_node; nodelist:a_node);
var
l4:integer;
l5:integer;
aux5:integer;
aux4:integer;
rid_curr:integer;
id_curr:integer;
begin
  l4:=length(robotlist);
  if l4>0 then
  begin
  for aux4:=0 to l4-1 do
  begin
    rid_curr:=robotlist[aux4].id_robot;
    if rid_curr=r_id then
    begin
     setlength(robotlist[aux4].current_nodes,1);
     robotlist[aux4].current_nodes[0]:=id;
     l5:=length(nodelist);
     for aux5:=0 to l5-1 do
     begin
       id_curr:=nodelist[aux5].id;
       if id_curr=id then
       begin
         robotlist[aux4].pos_X:=nodelist[aux5].pos_X;
         robotlist[aux4].pos_Y:=nodelist[aux5].pos_Y;
       end;
     end;
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
         //colour.
        newcube.Material.FrontProperties.Ambient:='clrBrown';
      end;
 end;
 end;

function get_index_node(nodelist:a_node; id:integer):integer;
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
       i_curr:=nodelist[aux1].id;
       if i_curr=id then
          begin
            i_max:=aux1;
          end;
      end;
      get_index_node:=i_max+1;
  end;

Function check_if_link_exists (n1:integer; n2:integer; nodelist:a_node):integer;

var
   l1:integer;
   l2:integer;
   n_curr:integer;
   ntl:integer;
   count:integer;
   aux4:integer;
   aux5:integer;
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

procedure create_link_between (n1:integer; n2:integer; nodelist:a_node; dist:Double);
var
  id:integer;
  l1:integer;
  l2:integer;
  n_curr:integer;
  c:integer;
  aux4:integer;
  aux5:integer;
begin
  c:=check_if_link_exists(n1,n2,nodelist);
  if c=0 then
  begin
  id:=get_max_id_link(nodelist);
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

function read_xml():a_node;
var
  nodelist:array of node_full;
  Doc: TXMLDocument;
  Nodes: TDOMNodeList;
  Node: TDOMNode;
  Links:TDOMNodeList;
  Link:TDOMNode;
  id: TDOMNode;
  x: TDOMNode;
  y: TDOMNode;
  def:TDOMNode;
  nl:TDOMNode;
  idl: TDOMNode;
  n2:TDOMNode;
  n1:TDOMNode;
  dist:TDOMNode;
  i: integer;
  id_value:string;
  x_value:string;
  y_value:string;
  def_value:string;
  nl_value:string;
  n2_value:string;
  n1_value:string;
  idl_value:string;
  dist_value:string;
  l1:integer;
  n1_i:integer;
  n2_i:integer;
  dist_d:Double;

begin

  ReadXMLFile(Doc, 'C:\Users\diogo\Desktop\pascal\read_xml\test.xml');
  Nodes:= Doc.GetElementsByTagName('Node');
  for i:= 0 to Nodes.Count - 1 do
  begin
     Node:= Nodes[i];
     id:=Node.Attributes.Item[0];
     id_value:=id.NodeValue;
     x:=Node.FindNode('x');
     x_value:=x.FirstChild.NodeValue;
     y:=Node.FindNode('y');
     y_value:=y.FirstChild.NodeValue;
     def:=Node.FindNode('Defined');
     def_value:=def.FirstChild.NodeValue;
     nl:=Node.FindNode('Number_of_Links');
     nl_value:=nl.FirstChild.NodeValue;
     l1:=length(nodelist);
     setlength(nodelist,l1+1);
     nodelist[l1].id:=strtoint(id_value);
     nodelist[l1].pos_X:=strtofloat(x_value);
     nodelist[l1].pos_y:=strtofloat(y_value);
     nodelist[l1].defined:=strtoint(def_value);
     nodelist[l1].number_of_links:=strtoint(nl_value);
  end;
   Links:= Doc.GetElementsByTagName('Link');
   for i:= 0 to Links.Count - 1 do
   begin
      Link:=Links[i];
      idl:=Link.Attributes.Item[0];
      idl_value:=idl.NodeValue;
      n1:=Link.FindNode('Node1_Id');
      n1_value:=n1.FirstChild.NodeValue;
      n1_i:=strtoint(n1_value);
      n2:=Link.FindNode('Node2_Id');
      n2_value:=n2.FirstChild.NodeValue;
      n2_i:=strtoint(n2_value);
      dist:=Link.FindNode('Distance');
      dist_value:=dist.FirstChild.NodeValue;
      dist_d:=strtofloat(dist_value);
      create_link_between(n1_i,n2_i,nodelist,dist_d);
   end;
    read_xml:=nodelist;
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

procedure write_xml_Simtwo (scale:integer; nodelist:a_node; width_line:double);

var
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
 l_done:array of integer;
 Doc: TXMLDocument;                                  // variable to document
 RootNode, parentNode, nofilho: TDOMNode;                    // variable to nodes
begin
 // Create a document
 Doc := TXMLDocument.Create;

 // Create a root node
 RootNode := Doc.CreateElement('track');
 Doc.Appendchild(RootNode);      // save root node


  // Create a defines node
  RootNode:= Doc.DocumentElement;
  parentNode := Doc.CreateElement('defines');
  RootNode.Appendchild(parentNode);                          // save parent node

  //create the conts atributes and node
  parentNode := Doc.CreateElement('conts');                // create a child node
  TDOMElement(parentNode).SetAttribute('name','field_length');     // create atributes
  TDOMElement(parentNode).SetAttribute('value',StringReplace(FloatToStr(3*scale),',','.',[rfReplaceAll, rfIgnoreCase]));
  RootNode.ChildNodes.Item[0].AppendChild(parentNode);       // insert child node in respective parent node

  //create the conts atributes and node
  parentNode := Doc.CreateElement('conts');                // create a child node
  TDOMElement(parentNode).SetAttribute('name','field_width');     // create atributes
  TDOMElement(parentNode).SetAttribute('value',StringReplace(FloatToStr(2*scale),',','.',[rfReplaceAll, rfIgnoreCase]));
  RootNode.ChildNodes.Item[0].AppendChild(parentNode);       // insert child node in respective parent node

  //create the conts atributes and node
  parentNode := Doc.CreateElement('conts');                // create a child node
  TDOMElement(parentNode).SetAttribute('name','ground');     // create atributes
  TDOMElement(parentNode).SetAttribute('value',StringReplace(FloatToStr(0.001),',','.',[rfReplaceAll, rfIgnoreCase]));
  RootNode.ChildNodes.Item[0].AppendChild(parentNode);       // insert child node in respective parent node

 count:=1;
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

            // Create a link line
            RootNode:= Doc.DocumentElement;
            parentNode := Doc.CreateElement('line');
            RootNode.Appendchild(parentNode);                          // save parent node

            //calculate angle of line
            //declive:=(y2-y1)/(x2-x1);
            angle:=arctan2((y2-y1),(x2-x1));


            //create the colour atributes and node
            parentNode := Doc.CreateElement('color');                // create a child node
            TDOMElement(parentNode).SetAttribute('rgb24','8F8F8F');     // create atributes
            RootNode.ChildNodes.Item[count].AppendChild(parentNode);       // insert child node in respective parent node

            //create the position atributes and node
            parentNode := Doc.CreateElement('position');                // create a child node
            TDOMElement(parentNode).SetAttribute('x',StringReplace(FloatToStr(x1),',','.',[rfReplaceAll, rfIgnoreCase]));     // create atributes
            TDOMElement(parentNode).SetAttribute('y',StringReplace(FloatToStr(y1),',','.',[rfReplaceAll, rfIgnoreCase]));
            TDOMElement(parentNode).SetAttribute('z','ground');
            TDOMElement(parentNode).SetAttribute('angle',StringReplace(FloatToStr(RadToDeg(angle)),',','.',[rfReplaceAll, rfIgnoreCase]));
            RootNode.ChildNodes.Item[count].AppendChild(parentNode);       // insert child node in respective parent node


             //create the position atributes and node
            parentNode := Doc.CreateElement('size');                // create a child node
            TDOMElement(parentNode).SetAttribute('width',StringReplace(FloatToStr(width_line),',','.',[rfReplaceAll, rfIgnoreCase]));     // create atributes
            TDOMElement(parentNode).SetAttribute('length',StringReplace(FloatToStr(dist),',','.',[rfReplaceAll, rfIgnoreCase]));
            RootNode.ChildNodes.Item[count].AppendChild(parentNode);       // insert child node in respective parent node

            count:=count+1;

            l3:=length(l_done);
            setlength(l_done,l3+1);
            l_done[l3]:=l_id;
                 end;
     end;
    end;


  writeXMLFile(Doc, 'track.xml');                     // write to XML
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

procedure Create_robots( robotlist:r_node; nodelist:a_node; x:Double; y:Double; scale:integer);
var
l4:integer;
aux4:integer;
count:integer;
max_id:integer;
id:integer;

begin
count:=1;
l4:=length(robotlist);
setlength(robotlist,l4+1);
max_id:=get_max_robotid(robotlist);
robotlist[l4].id_robot:=max_id+count;
id:=get_closest_node_id(nodelist, x, y, scale);
update_robot_inicial_position(max_id+count, id, robotlist, nodelist);
end;

{ TForm1 }

procedure TForm1.FormShow(Sender: TObject);
begin
    full_nodelist:=read_xml();
    l1:=length(full_nodelist);
      for aux1:=0 to l1-1 do
      begin
       i_curr:=full_nodelist[aux1].id;
       x:=full_nodelist[aux1].pos_X;
       y:=full_nodelist[aux1].pos_y;
       StringGrid1.InsertRowWithValues(1,[inttostr(i_curr), floattostr(x), floattostr(Y)]);
      end;
      for aux1:=0 to l1-1 do
      begin
        i_curr:=full_nodelist[aux1].id;
        l2:=length(full_nodelist[aux1].links);
        for aux2:=0 to l2-1 do
        begin
         id_l:=full_nodelist[aux1].links[aux2].id_l;
         ntl:=full_nodelist[aux1].links[aux2].node_to_link;
         dist:=full_nodelist[aux1].links[aux2].distance;

        end;
      end;
      ReadXMLFile(Doc, 'C:\Users\diogo\Desktop\pascal\read_xml\test.xml');
      Nodes:= Doc.GetElementsByTagName('Node');
      Node:= Nodes[0];
      id:=Node.Attributes.Item[0];
      write_xml_Simtwo(10,full_nodelist,0.25);
      //newline1:=TGLLines.CreateAsChild(GLScene2.Objects);
      //newline1.LineWidth:=25;
      //newline1.AddNode(1,25,0);
      //newline1.AddNode(300,25,0);
      //GLScene2.Objects.addchild(newline1);
      Print_map_in_GLS(full_nodelist,10,GLScene2,GLDummyCube3,200);
      l2:=length(robots);
      if l2>0 then
      begin
      for aux1:=0 to l1-1 do
      begin
         id_l:=robots[aux1].id_robot;
         x_r:=robots[aux1].pos_X;
         y_r:=robots[aux1].pos_y;
         StringGrid2.InsertRowWithValues(1,[inttostr(id_l), floattostr(x_r), floattostr(y_r)]);
        end;
      end;
end;

procedure TForm1.Button1Click(Sender: TObject);
begin
  x_r:=strtofloat(Labelededit2.Text);
  y_r:=strtofloat(Labelededit3.Text);
  //Create_robots(robots,full_nodelist,x_r,y_r,10);
l4:=length(robots);
setlength(robots,l4+1);
max_id:=get_max_robotid(robots);
robots[l4].id_robot:=max_id+1;
id_r:=get_closest_node_id(full_nodelist, x_r, y_r, 200);
update_robot_inicial_position(max_id+1, id_r, robots, full_nodelist);
  l2:=length(robots);
      if l2>0 then
      begin
      for aux1:=0 to l2-1 do
      begin
         id_l:=robots[aux1].id_robot;
         x_r:=robots[aux1].pos_X;
         y_r:=robots[aux1].pos_y;
         StringGrid2.InsertRowWithValues(1,[inttostr(id_l), floattostr(x_r), floattostr(y_r)]);
        end;
      end;
  print_robot_position_GLS(robots,200,GLScene2,GLDummyCube3,25,25);
end;

procedure TForm1.GLCadencer1Progress(Sender: TObject; const deltaTime,
  newTime: Double);
begin
   if ((mx <> mx2) or (my <> my2)) then
  begin
    GLCamera1.MoveAroundTarget(my - my2, mx - mx2);
    mx := mx2;
    my := my2;
  end;
end;

procedure TForm1.GLSceneViewer1MouseDown(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin
  mx := X;
  my := Y;
  mx2 := X;
  my2 := Y;
end;

procedure TForm1.GLSceneViewer1MouseMove(Sender: TObject; Shift: TShiftState;
  X, Y: Integer);
begin
   if ssLeft in Shift then
  begin
    mx2 := X;
    my2 := Y;
  end;
end;

procedure TForm1.GLSceneViewer1MouseWheel(Sender: TObject; Shift: TShiftState;
  WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
begin
   GLCamera1.AdjustDistanceToTarget(Power(1.025, WheelDelta / 300));
end;


end.

