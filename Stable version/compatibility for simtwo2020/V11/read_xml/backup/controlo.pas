unit controlo;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, lNetComponents, Forms, Controls, Graphics,
  Dialogs, StdCtrls, ExtCtrls, lNet, TEAstar, laz2_DOM, laz2_XMLRead, math, Utils3DS,Utils;

const
  THRESHOLD_DIST = 0.033;//0.025;
  THRESHOLD_ANGLE = 0.34;   //0.33
  //THRESHOLD_ANGLE_BACK = 0.3;
  CELLSCALE = 1;
  VNOM = 5.5 ;
  WNOM = 40;  //20
  GANHO_DIST = 240;    //320
  GANHO_THETA = 60;    //40
  GANHO_DIST_CIRCLE = 340;  //340
  GANHO_THETA_CIRCLE = 30;  //30
  GANHO_DIST_CIRCLE_BACK = 260;
  GANHO_THETA_CIRCLE_BACK = 80;
  RAIO_CURVATURA = CELLSCALE;

  //GANHOS DIMENSIONADOS PARA VNOM=1.3
  //240:60:400:40:480:30

type

  { TFControlo }

  Coordinates = record
    x, y: double;
  end;

   Coms_flaws_per_robot =record
     isactive:integer;
     isdetecting:integer;
     active_consecutive_hits:integer;
     n_flaws:integer;
     in_node:array of integer;
     out_node:array of integer;
     detected_nodes:array of array of integer;
     flaw_ind:integer;
   end;

  TFControlo = class(TForm)
    Edit10: TEdit;
    Edit2: TEdit;
    Edit3: TEdit;
    Edit4: TEdit;
    Edit5: TEdit;
    Edit6: TEdit;
    Edit7: TEdit;
    Edit8: TEdit;
    Edit9: TEdit;
    LabeledEdit1: TLabeledEdit;
    SendButton: TButton;
    Edit1: TEdit;
    TimerSend: TTimer;
    udpCom: TLUDPComponent;
    //procedure Coms_timeout_timerTimer(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure SendButtonClick(Sender: TObject);
    procedure TimerSendTimer(Sender: TObject);
    procedure udpComReceive(aSocket: TLSocket);
  private
    { private declarations }
  public
    { public declarations }
  end;

   r_node=TEAstar.r_node;
   m_node=array[0..NUMBER_ROBOTS-1] of integer;
   i_array=array of integer;
procedure InitialPointsForAllRobots(var agvs:r_node);
procedure ChangeRobotPriorities(var Map:TAStarMap;var agvs:r_node);
procedure InverseValidationOfPriorities(var Map:TAStarMap;var agvs:r_node;var CaminhosAgvs:Caminhos);
procedure TEArun(var Map:TAStarMap;var agvs:r_node;var CaminhosAgvs:Caminhos);
function DistToReference(robot:integer;xCam:DOUBLE;yCam:double):double;
function AngleToReference(robot:integer;xCam:double;yCam:double;thetaCam:double):double;
function Signal(value:integer):integer;
procedure UpdateThetaDest(robot:integer;thetaCam:double);
procedure UpdateThetaDestPi(robot:integer;thetaCam:double);
procedure UpdateThetaDestToMoveBack(robot:integer;thetaCam:double);
procedure UpdateThetaDestAfterPiRotation(robot:integer;thetaCam:double);
procedure UnpackUDPmessage(var xCam:array of double;var yCam:array of double; var thetaCam:array of double; var id_rob:integer ;data:string);
procedure UpdateInitialPoints(var xCam:array of double;var yCam:array of double; var thetaCam:array of double; var id_rob:integer );
procedure UpdateSubmissions(var agvs:r_node;i:integer);

var
  FControlo: TFControlo;
  MessageInitialPositions: string;
  MessageVelocities: string;
  MessageVelocities1: string;
  Map: TAStarMap;
  agvs: r_node;
  CaminhosAgvs: Caminhos;
  CaminhosAgvs_af: Caminhos;
  CaminhosAgvs_a: Caminhos;
  Ca: array[0..NUMBER_ROBOTS-1] of integer;
  flagMessageInitialPositions: boolean;
  flagVelocities: boolean;
  xDest: array[0..NUMBER_ROBOTS-1] of double;
  yDest: array[0..NUMBER_ROBOTS-1] of double;
  thetaDest: array[0..NUMBER_ROBOTS-1] of double;
  directionDest: array[0..NUMBER_ROBOTS-1] of integer;
  contador: double;
  linearVelocities: array[0..NUMBER_ROBOTS-1] of double;
  angularVelocities: array[0..NUMBER_ROBOTS-1] of double;
  followLine: array[0..NUMBER_ROBOTS-1] of boolean;
  followCircle: array[0..NUMBER_ROBOTS-1] of boolean;
  rotate: array[0..NUMBER_ROBOTS-1] of boolean;
  robotNoPlan: integer;
  Doc: TXMLDocument;
  directionToFollow: array[0..NUMBER_ROBOTS-1] of integer;
  target: array[0..NUMBER_ROBOTS-1] of boolean;
  rotationCenter: array[0..NUMBER_ROBOTS-1] of Coordinates;
  s: array[0..NUMBER_ROBOTS-1] of Coordinates;
  dist,angle:double;
  wf: array[0..NUMBER_ROBOTS-1] of double;
  flagChange: boolean;
  ListPriorities: array[0..NUMBER_ROBOTS-1] of integer;
  //mantém-se a ordem inicial dos robôs e guarda-se a prioridade atual
  totalTrocas: integer;
  totalValidations: integer;
  ct:integer;
  c_cnodes:integer;
  init:array[0..NUMBER_ROBOTS-1] of integer;
  ind_robo: integer;
  id_rob:integer;
  a_c_flaw:array[0..NUMBER_ROBOTS-1] of integer;
  coms_count:array[0..NUMBER_ROBOTS-1] of integer;
  pre_coms_count:array[0..NUMBER_ROBOTS-1] of integer;
  robots_flaws:array [0..NUMBER_ROBOTS-1] of  Coms_flaws_per_robot;
  t1,t2,tick_p,s_time:QWord;
  total_seconds:longint;
  timestamp_coms:array[0..NUMBER_ROBOTS-1] of longint;
  type_of_movement:array[0..NUMBER_ROBOTS-1] of integer;
  current_step:array[0..NUMBER_ROBOTS-1] of integer;
  step_complete:array[0..NUMBER_ROBOTS-1] of integer;
  ghost_nodes:array[0..NUMBER_ROBOTS-1] of integer;
  aux_exit_flaw_node:integer;
  b_pathsend:integer;
implementation
 uses
   unit1,unit2;
{$R *.lfm}

{ Functions/Procedures }
function round2(const Number: extended; const Places: longint): extended;
var t: extended;
begin
   t := power(10, places);
   round2 := round(Number*t)/t;
end;
function blocked_node(var n1:integer; n2:integer):integer;
var
  n_id,ntl:integer;
  l1,l2:integer;
  aux1,aux2:integer;
  c:integer;
begin
   l1:=length(form1.full_nodelist);
   c:=0;
   for aux1:=0 to l1-1 do
   begin
     n_id:=form1.full_nodelist[aux1].id;
     if n_id=n2 then
     begin
       l2:=length(form1.full_nodelist[aux1].links);
       for aux2:=0 to l2-1 do
       begin
         ntl:=form1.full_nodelist[aux1].links[aux2].node_to_link;
         if n1=ntl then
         begin
             c:=1;
         end;
       end;
     end;
   end;
   blocked_node:=c;
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
function getmaxid(map:TAStarMap):integer;
var
aux1:integer;
id,id_m,l1:integer;
begin
  l1:=length(map.TEA_GRAPH);
  id_m:=0;
  for aux1:=0 to l1-1 do
  begin
    id:=map.TEA_GRAPH[aux1][0].id;
    if id>id_m then
    begin
      id_m:=id;
    end;
  end;
    getmaxid:=id_m;
end;

function getmaxlid(map:TAStarMap):integer;
var
aux1,aux2:integer;
id,lid,lid_m,l1,l2:integer;
begin
  l1:=length(map.TEA_GRAPH);
  lid_m:=0;
  for aux1:=0 to l1-1 do
  begin
    l2:=length(map.TEA_GRAPH[aux1][0].links);
    id:=map.TEA_GRAPH[aux1][0].id;
     for aux2:=0 to l2-1 do
     begin
         lid:=map.TEA_GRAPH[aux1][0].links[aux2].id_l;
         if lid>lid_m then
         begin
           lid_m:=lid;
         end;
     end;
  end;
    getmaxlid:=lid_m;
end;
function getdisttonode (map:TAStarMap; i:integer; x:double; y:double):Double;
var
l1,aux1,id1:integer;
dist,aux2,aux3,aux4,d_x,d_y,x1,y1:double;
begin
  l1:=length(form1.map.TEA_GRAPH);
  dist:=99999999999;
  for aux1:=0 to l1-1 do
  begin
    id1:=form1.map.TEA_GRAPH[aux1][0].id;
    if id1=i then begin
      x1:= form1.map.TEA_GRAPH[aux1][0].pos_X;
      y1:= form1.map.TEA_GRAPH[aux1][0].pos_Y;
      d_x:=x-x1;
      d_y:=y-y1;
      aux2:=d_x*d_x;
      aux3:=d_y*d_y;
      aux4:=aux2+aux3;
      dist:=sqrt(aux1);
    end;
  end;
   getdisttonode:=dist;
end;



function create_temp_node(id1:integer; id2:integer; X:Double; Y:Double; r:integer):integer;
var
  s,l1,l2:integer;
  id_aux,idl_aux1,idl_aux2:integer;
  map_aux:TAStarMap;
begin
  l1:=length(form1.map.TEA_GRAPH);
  map_aux:=form1.map;
  setlength(map_aux.TEA_GRAPH, l1+1,NUM_LAYERS);
  setlength(map_aux.GraphState, l1+1,NUM_LAYERS);
  setlength(map_aux.HeapArray.data, (l1+1)*NUM_LAYERS);
  form1.map:=map_aux;
  l2:=length(form1.map.TEA_GRAPH);
  id_aux:=getmaxid(form1.map)+1;
  idl_aux1:=getmaxlid(form1.map)+1;
  idl_aux2:=idl_aux1+1;

  for s:=0 to 1 do
  begin
      form1.map.TEA_GRAPH[l1][s].id:=id_aux;
      form1.map.TEA_GRAPH[l1][s].pos_X:=X;
      form1.map.TEA_GRAPH[l1][s].pos_Y:=Y;
      setlength(form1.map.TEA_GRAPH[l1][s].links,2);
      form1.map.TEA_GRAPH[l1][s].links[0].id_l:=idl_aux1;
      form1.map.TEA_GRAPH[l1][s].links[0].node_to_link:=id1;
      form1.map.TEA_GRAPH[l1][s].links[0].distance:=getdisttonode(form1.map,id1,X,Y);
      form1.map.TEA_GRAPH[l1][s].links[1].id_l:=idl_aux2;
      form1.map.TEA_GRAPH[l1][s].links[1].node_to_link:=id2;
      form1.map.TEA_GRAPH[l1][s].links[1].distance:=getdisttonode(form1.map,id2,X,Y);
      form1.map.GraphState[l1][s]:=VIRGIN;
  end;
  c_cnodes:=c_cnodes+1;
  ghost_nodes[r]:=id_aux;
  create_temp_node:=id_aux;
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
end;
  get_closest_node_id:=id_min;
end;
function getdirlink (dx:double; dy:double):integer;
begin
  if ((dx = 0) and (dy >0)) then begin getdirlink := 0; end
  else if ((dx = 0) and (dy <0)) then begin getdirlink := 4; end
  else if ((dx > 0) and (dy = 0)) then begin getdirlink := 2; end
  else if ((dx <0) and (dy = 0)) then begin getdirlink := 6; end
  else if ((dx >0) and (dy >0)) then begin getdirlink := 1; end
  else if ((dx >0) and (dy <0)) then begin getdirlink := 3; end
  else if ((dx <0) and (dy >0)) then begin getdirlink := 7; end
  else if ((dx <0) and (dy <0)) then begin getdirlink := 5; end;
end;




function get_linked_node(nodelist:a_node; x:Double; y:Double; scale:integer; id1:integer):integer;
var
    l4,l2,aux1,aux2,aux3,n_curr,id2_aux,id2:integer;
    x1,y1,x2,y2:double;
    diff1x,diff1y,diff2x,diff2Y:double;
    dir1,dir2:integer;
begin
  l4:=length(nodelist);
  id2:=0;
   for aux1:=0 to l4-1 do
   begin
     n_curr:=nodelist[aux1].id;
     if n_curr=id1 then
        begin
        l2:=length(nodelist[aux1].links);
        x1:=nodelist[aux1].pos_X;
        y1:=nodelist[aux1].pos_Y;
        diff1x:=(x-x1);
        diff1y:=(y-y1);
        dir1:=getdirlink(diff1x,diff1y);
        for aux2:=0 to l2-1 do
        begin
           id2_aux:=nodelist[aux1].links[aux2].node_to_link;
           for aux3:=0 to l4-1 do
           begin
              x2:=nodelist[aux3].pos_X;
              y2:=nodelist[aux3].pos_Y;
           end;
            diff2x:=(x2-x1);
            diff2y:=(y2-y1);
            dir2:=getdirlink(diff2x,diff2y);
            if dir2=dir1 then
               begin
                id2:=id2_aux
               end;
        end;
   end;
   end;
   get_linked_node:=id2;
end;
 procedure delete_ghost_node(i:integer);
 var
     s1,aux2,aux1:integer;
     map_aux:TAStarMap;
 begin
      for aux1:=0 to length(form1.map.TEA_GRAPH) do
      begin
        if form1.map.TEA_GRAPH[aux1][0].id=i then begin
            for aux2:=aux1 to length(form1.map.TEA_GRAPH)-1 do begin
                for s1:=0 to 1 do begin
                    form1.map.TEA_GRAPH[aux2][s1]:=form1.map.TEA_GRAPH[aux2+1][s1];
                    form1.map.GraphState[aux2][s1]:= form1.map.GraphState[aux2+1][s1];
                end;
            end;
              map_aux:=form1.map;
              setlength(map_aux.TEA_GRAPH, length(form1.map.TEA_GRAPH)-1,NUM_LAYERS);
              setlength(map_aux.GraphState, length(form1.map.TEA_GRAPH)-1,NUM_LAYERS);
              setlength(map_aux.HeapArray.data, (length(form1.map.TEA_GRAPH)-1)*NUM_LAYERS);
              form1.map:=map_aux;
        end;
      end;
 end;

function updaterobotnode(map:TAStarMap; nodelist:a_node; x:Double; y:Double; scale:integer; r:integer):integer;
var
    aux1,l4,idf:integer;
    n_curr:integer;
    x_p:Double;
    y_p:Double;
    threshold_d:Double;
    id1_aux,id2_aux:integer;
begin
  l4:=length(nodelist);
  idf:=0;
  threshold_d:=0;
  if ghost_nodes[r]<>0 then begin
     delete_ghost_node(ghost_nodes[r]);
  end;
  for aux1:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux1].id;
     x_p:=nodelist[aux1].pos_X*scale+threshold_d;
     y_p:=nodelist[aux1].pos_Y*scale+threshold_d;
     if ((x=x_p) and (y=y_p)) then
        begin
         idf:=n_curr;
        end;
  end;
  if idf=0 then
     begin
       id1_aux:=get_closest_node_id(nodelist,x,y,scale);
       id2_aux:=get_linked_node(nodelist,x,y,scale,id1_aux);
       if id2_aux=0 then begin
         idf:=create_temp_node(id1_aux,id2_aux,x,y,r);
       end;
       idf:=create_temp_node(id1_aux,id2_aux,x,y,r);
     end  else begin
          ghost_nodes[r]:=0;
     end;
 updaterobotnode:=idf;
end;
function get_remaining_path_nodes(CaminhosAgvs:Caminhos; i:integer;n:integer):i_array;
var
 s1,s2,aux1,aux2,c_a:integer;
 r_aux:array of integer;
begin
  s1:=length(CaminhosAgvs[i].coords);
  for aux1:=0 to s1-1 do begin
     if ((CaminhosAgvs[i].coords[aux1].node=n) and (n<>0)) then begin
        for aux2:=aux1 to s1-1 do begin
           if  CaminhosAgvs[i].coords[aux2].node<>0 then begin
             s2:=length(r_aux);
             setlength(r_aux,s2+1);
             r_aux[s2]:= CaminhosAgvs[i].coords[aux2].node;
           end;
        end;
     end;
  end;
  get_remaining_path_nodes:=r_aux;
end;

function save_flaw_location(CaminhosAgvs:Caminhos; agvs:r_node; i:integer; nodelist:a_node; s:integer):Coms_flaws_per_robot;
var
   n_curr, at,idf,aux1,aux2,aux_id,s1,s2,s4:integer;
   aux_robots_flaws: Coms_flaws_per_robot;
begin
   aux_robots_flaws:=robots_flaws[i];
   aux_robots_flaws.isactive:=1;
   aux2:=aux_robots_flaws.active_consecutive_hits;
   aux_robots_flaws.active_consecutive_hits:=aux2+1;
   s4:=length(nodelist);
   idf:=0;
   for aux1:=0 to s4-1 do
   begin
     n_curr:=nodelist[aux1].id;
     if n_curr=agvs[i].inicial_node then
        begin
         idf:=n_curr;
        end;
  end;
   if idf=0 then begin
   aux_id:=get_linked_node(nodelist,agvs[i].pos_X,agvs[i].pos_y,1,CaminhosAgvs[i].coords[s].node);
   end else begin
    aux_id:=idf;
   end;
   s1:=length(aux_robots_flaws.in_node);
   at:=0;
   if s1>0 then begin
   for aux1:=0 to s1-1 do begin
       if aux_robots_flaws.in_node[aux1]=aux_id then begin
         at:=1;
         aux_robots_flaws.flaw_ind:=aux1;
       end;
   end;
   end;
   if at=0 then begin
      setlength(aux_robots_flaws.in_node, s1+1);
      aux_robots_flaws.in_node[s1]:=aux_id;
      aux_robots_flaws.isdetecting:=1;
      aux_robots_flaws.n_flaws:= aux_robots_flaws.n_flaws+1;
      s2:=length(aux_robots_flaws.detected_nodes);
      setlength(aux_robots_flaws.detected_nodes,s2+1);
      aux_robots_flaws.detected_nodes[s2]:=get_remaining_path_nodes(CaminhosAgvs,i,aux_id);
      aux_robots_flaws.flaw_ind:=s1;
   end;
    CaminhosAgvs_af[i]:=CaminhosAgvs[i];
   save_flaw_location:=aux_robots_flaws;
end;
function update_flaw_path(n:integer;ni:integer ; i:integer;CaminhosAgvs:Caminhos):i_array;
var
 at,s1,s2,aux1,aux2,c_a,a1,a2,c1,c2,diff:integer;
 r_aux:array of integer;
begin
  s1:=length(CaminhosAgvs[i].coords);
  c1:=99999;
  c2:=99999;
  for aux1:=0 to s1-1 do begin
     if ((CaminhosAgvs[i].coords[aux1].node=ni) and (c1=99999)) then begin
       c1:=aux1;
     end else if ((CaminhosAgvs[i].coords[aux1].node=n) and (c2=99999)) then begin
       c2:=aux1;
     end;
  end;
  if c1 > c2 then begin
    diff:=c2-c1;
  end;
  diff:=c2-c1;
  setlength(r_aux,diff+1);
  c_a:=0;
  if diff=0 then begin
  s2:=length(form1.crit_nodes);
  at:=0;
  for aux1:=0 to s2-1 do begin
  if form1.crit_nodes[aux1]=n then begin
    at:=1
  end else begin
      at:=0;
  end;
  end;
  if at=1 then begin
        a1:=99999;
       a2:=99999;
     for aux2:=0 to s1-1 do begin
     if ((CaminhosAgvs[i].coords[aux1].node=n) and (a1=99999)) then begin
       a1:=aux1;
     end else if ((CaminhosAgvs[i].coords[aux1].node=n) and (a2=99999) and (a2<>a1)) then begin
       a2:=aux1;
     end;
     end;
     diff:=a2-a1;
     setlength(r_aux,diff+1);
     c_a:=0;
    for aux2:=a1 to a2 do begin
       r_aux[c_a]:= CaminhosAgvs[i].coords[aux2].node;
       c_a:=c_a+1;
    end;
  end else begin
    r_aux[c_a]:= CaminhosAgvs[i].coords[c2].node;
  end;
  end else begin
  for aux2:=c1 to c2 do begin
     r_aux[c_a]:= CaminhosAgvs[i].coords[aux2].node;
     c_a:=c_a+1;
  end;
  end;

  update_flaw_path:=r_aux;
end;

function save_exit_node(agvs:r_node;i:integer):Coms_flaws_per_robot;
var
    aux_robots_flaws: Coms_flaws_per_robot;
    aux_id,aux1,s1,s4,n_curr,idf:integer;
begin
   aux_robots_flaws:=robots_flaws[i];
   s1:=length(aux_robots_flaws.out_node);
   setlength(aux_robots_flaws.out_node,s1+1);
   s4:=length(form1.full_nodelist);
   idf:=0;
   for aux1:=0 to s4-1 do
   begin
     n_curr:=form1.full_nodelist[aux1].id;
     if n_curr=agvs[i].inicial_node then
        begin
         idf:=n_curr;
        end;
  end;
   if idf=0 then begin
   aux_id:=get_linked_node(form1.full_nodelist,agvs[i].pos_X,agvs[i].pos_y,1,CaminhosAgvs_af[i].coords[1].node);
   end else begin
    aux_id:=idf;
   end;
   aux_robots_flaws.out_node[s1]:=aux_id;
   aux_robots_flaws.detected_nodes[s1]:=update_flaw_path(aux_robots_flaws.out_node[s1],aux_robots_flaws.in_node[s1],i,CaminhosAgvs_af);
   save_exit_node:=aux_robots_flaws;
end;

function update_afected_nodes(n:integer;ni:integer ; i:integer;CaminhosAgvs:Caminhos; r_flaw:Coms_flaws_per_robot):i_array;
var
    s1,s2,aux1,aux2,c_a,c1,c2,diff:integer;
    r_aux:array of integer;
begin
     s1:=length(CaminhosAgvs[i].coords);
     s2:=length(r_flaw.detected_nodes[r_flaw.flaw_ind]);
     for aux1:=0 to s1-1 do begin
        if CaminhosAgvs[i].coords[aux1].node=ni then begin
          c1:=aux1;
        end else if CaminhosAgvs[i].coords[aux1].node=n then begin
          c2:=aux1;
        end;
     end;
     diff:=c2-c1;
     setlength(r_aux,s2+diff);
     c_a:=s2-1;
     for aux2:=c1 to c2 do begin
        r_aux[c_a]:= CaminhosAgvs[i].coords[aux2].node;
        c_a:=c_a+1;
     end;

     update_afected_nodes:=r_aux;
end;

function  checkfornode (nodelist:a_node; x:Double; y:Double; scale:integer):integer;
 var
    aux1,l4,idf:integer;
    n_curr:integer;
    x_p:Double;
    y_p:Double;
    threshold_d:Double;
    id1_aux,id2_aux:integer;
begin
  l4:=length(nodelist);
  idf:=0;
  threshold_d:=0;
  for aux1:=0 to l4-1 do
  begin
     n_curr:=nodelist[aux1].id;
     x_p:=nodelist[aux1].pos_X*scale+threshold_d;
     y_p:=nodelist[aux1].pos_Y*scale+threshold_d;
     if ((x=x_p) and (y=y_p)) then
        begin
         idf:=n_curr;
        end
  end;
  checkfornode:=idf;
end;
procedure removenodes(map:TAStarMap;c:integer);
var
l1:integer;
begin
 l1:=length(map.TEA_GRAPH);
 setlength(map.TEA_GRAPH,l1-c,NUM_LAYERS);
end;

function get_steps(var CaminhosAgvs:Caminhos;i:integer):integer;
 var
 l1,aux1,aux2:integer;

 Begin
l1:=length((CaminhosAgvs[i].coords));
aux2:=0;
for aux1:=0 to l1-1 do begin
if ((getXcoord(CaminhosAgvs[i].coords[aux1].node)<3) and (getYcoord(CaminhosAgvs[i].coords[aux1].node)<3)) then
           begin
           aux2:=aux2+1
           end;
   end;
get_steps:=aux2;
end;

procedure InitialPointsForAllRobots(var agvs:r_node);
var
  v:integer;
begin
    MessageInitialPositions := '';
    MessageInitialPositions := MessageInitialPositions + 'T' + IntToStr(NUMBER_ROBOTS);
    v:=0;
    while v < NUMBER_ROBOTS do begin
      ListPriorities[v]:=v;
      MessageInitialPositions := MessageInitialPositions + 'R' + 'X' +
                                 floatToStr(round2(agvs[v].pos_X,3)) + 'Y' +
                                 floatToStr(round2(agvs[v].pos_Y,3)) + 'D' +
                                 floatToStr(agvs[v].Direction);
      xDest[v]:=agvs[v].pos_X;
      yDest[v]:=agvs[v].pos_Y;
      v:=v+1;
    end;
    MessageInitialPositions := MessageInitialPositions + 'F';
end;

function checkforpathcompletion(agvs:r_node; i:integer):integer;
var
  l1,aux1,r:integer;
begin
  r:=0;
  if agvs[i].inicial_node=agvs[i].target_node then
  begin
   r:=1;
  end;
  checkforpathcompletion:=r;
end;
function return_id_frompriority(var agvs: r_node; p:integer):integer;
var
 l1,aux1,r:integer;
Begin
  r:=0;
  l1:=length(agvs);
  for aux1:=0 to l1-1 do
  begin
    if p=agvs[aux1].InitialIdPriority then
    begin
         r:=aux1;
    end;
  end;
   return_id_frompriority:=r;
end;

function get_t_mov(CaminhosAgvs:Caminhos; i:integer):m_node;
var
  aux1:integer;
  aux_array:array[0..NUMBER_ROBOTS-1] of integer;
begin
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
    if CaminhosAgvs[aux1].coords[i-1].node=CaminhosAgvs[aux1].coords[i].node then
    begin
      if CaminhosAgvs[aux1].coords[i-1].direction=CaminhosAgvs[aux1].coords[i].direction then
      begin
         aux_array[aux1]:=1;//wait
      end else begin
         aux_array[aux1]:=2;//rotate
      end;
    end
    else begin
         aux_array[aux1]:=3; //go foward
    end;
  end;
  get_t_mov:=aux_array;
end;
function  check_step_comp(CaminhosAgvs:Caminhos;agvs: r_node;s:m_node): m_node;
var
  aux1:integer;
  res:array[0..NUMBER_ROBOTS-1] of integer;
begin
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
      if ((CaminhosAgvs[aux1].coords[s[aux1]].node=agvs[aux1].inicial_node) and (CaminhosAgvs[aux1].coords[s[aux1]].direction=agvs[aux1].Direction)) then
      begin
         res[aux1]:=1;
      end else begin
         res[aux1]:=0;
      end;
  end;
   check_step_comp:=res;
end;

function check_if_still_on_path(CaminhosAgvs:Caminhos;agvs: r_node;i:integer;t_d:double;s:m_node):integer;
var
tom,step_complete:array[0..NUMBER_ROBOTS-1] of integer;
aux_rx,aux_ry,aux_x,aux_y:double;
r,aux_step_complete:integer;
begin
  tom:=get_t_mov(CaminhosAgvs,1);
  r:=0;
  step_complete:=check_step_comp(CaminhosAgvs,agvs,s);
  aux_step_complete:=0;
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
      if step_complete[aux1]=1 then begin
         aux_step_complete:=1;
      end;
  end;
  aux_x:=round2(getXcoord(CaminhosAgvs[i].coords[s[aux1]].node),3);
  aux_y:=round2(getXcoord(CaminhosAgvs[i].coords[s[aux1]].node),3);
  aux_rx:=round2(agvs[i].pos_X,3);
  aux_ry:=round2(agvs[i].pos_y,3);
  if aux_step_complete=1 then begin
  if tom[i]=1 then
  begin
   if ((aux_rx=aux_x) and (aux_ry=aux_y) and  (agvs[i].Direction=CaminhosAgvs[i].coords[s[aux1]].direction)) then
   begin
        r:=0;
   end else begin
       r:=1;
   end;
  end else if tom[i]=2 then begin
  if ((aux_rx=aux_x) and (aux_ry=aux_y) and  ((agvs[i].Direction>=CaminhosAgvs[i].coords[s[aux1]].direction-1) and (agvs[i].Direction<=CaminhosAgvs[i].coords[s[aux1]].direction+1)) ) then
   begin
        r:=0;
   end else begin
       r:=1;
   end;
  end else if tom[i]=3 then begin
      if (((aux_rx>=aux_x-t_d) and(aux_rx<=aux_x+t_d)) and ((aux_ry>=aux_y-t_d) and (aux_ry<=aux_y+t_d)) and (agvs[i].Direction=CaminhosAgvs[i].coords[s[aux1]].direction) ) then
   begin
        r:=0;
   end else begin
       r:=1;
   end;
  end;
  end;
  check_if_still_on_path:=r;
end;

procedure ChangeRobotPriorities(var Map:TEAstar.TAStarMap;var agvs:r_node);
var
    aux_agv: TEAstar.Robot_Pos_info;
    p_id,aux1:integer;
begin
        noPath:=false;
        trocas:=trocas+1;
        p_id:=agvs[robotNoPlan].InitialIdPriority;
        if p_id>1 then
        begin
        aux1:=return_id_frompriority(agvs,p_id-1);
        agvs[robotNoPlan].InitialIdPriority:=p_id-1;
        agvs[aux1].InitialIdPriority:=p_id;
        robotNoPlan := A_starTimeGo(Map,CaminhosAgvs,agvs,MAX_ITERATIONS);
        end;
end;

procedure InverseValidationOfPriorities(var Map:TAStarMap;var agvs:r_node;var CaminhosAgvs:Caminhos);
var
    stepPath: integer;
    v,robo: integer;
    i,j,k,count,steps: integer;
    aux_agv: TEAstar.Robot_Pos_info;
    p_id,aux1:integer;
begin

     //inverse validation of the planning to avoid that a robot with lower priority
     //can reach the target early and intersect the path of a robot with higher
     //priority; the idea is detect this situations and change the priority again;
     //in this situation, after exchange priorities, plans again and allow to put
     //the target as obstacle in every layer to avoid undefined exchanges

     trocas:=0;
     robo:=NUMBER_ROBOTS-1;
     while robo > 0 do begin
       v:=robo-1;
       while v >= 0 do begin
         stepPath:=CaminhosAgvs[v].steps;
         while stepPath > 0 do begin
           i:=CaminhosAgvs[v].coords[stepPath].node;
           k:=CaminhosAgvs[v].coords[stepPath].steps;
           if ((blocked_node(i,agvs[robo].target_node)=1) and (k>CaminhosAgvs[robo].steps))
               or ((i=agvs[robo].target_node) and (k>CaminhosAgvs[robo].steps))
           then begin

              robotNoPlan:=robo;
              while ((robotNoPlan > 0) and (trocas < MAX_EXCHANGES)) do begin

                noPath:=false;
                trocas:=trocas+1;
                p_id:=agvs[robotNoPlan].InitialIdPriority;
                if p_id>1 then
                begin
                aux1:=return_id_frompriority(agvs,p_id-1);
                agvs[robotNoPlan].InitialIdPriority:=p_id-1;
                agvs[aux1].InitialIdPriority:=p_id;
                robotNoPlan := A_starTimeGo(Map,CaminhosAgvs,agvs,MAX_ITERATIONS);
                end;

                robotNoPlan := A_starTimeGo(Map,CaminhosAgvs,agvs,MAX_ITERATIONS);

              end;

           end;
           stepPath:=stepPath-1;
         end;
         v:=v-1;
       end;
       robo:=robo-1;
     end;

end;
procedure TEArun(var Map:TAStarMap;var agvs:r_node;var CaminhosAgvs:Caminhos);
var
    aux1:integer;
begin
    flagTargetOverlapInverse:=false;

    robotNoPlan := A_starTimeGo(Map,CaminhosAgvs,agvs,MAX_ITERATIONS);

    if ((robotNoPlan <> -1)) then begin
        trocas:=0;
        while ((robotNoPlan > 0) and (trocas < MAX_EXCHANGES)) do begin
              ChangeRobotPriorities(Map,agvs);
              flagChange:=true;
        end;
        totalTrocas:=trocas;
    end;

    trocas:=0;
    flagTargetOverlapInverse := true;
    InverseValidationOfPriorities(Map,agvs,CaminhosAgvs);
    if trocas > 0 then begin
        flagChange:=true;
    end;
    totalValidations:=trocas;
    removenodes(map,c_cnodes);
    c_cnodes:=0;
    for aux1:=0 to NUMBER_ROBOTS-1 do begin
        current_step[aux1]:=1;
    end;
    type_of_movement:=get_t_mov(CaminhosAgvs,1);
end;

function DistToReference(robot:integer;xCam:double;yCam:double):double;
begin
   if followLine[robot] = true then begin
       case directionDest[robot] of
       0: begin
               result:=xCam-(xDest[robot])*CELLSCALE;
          end;
       2: begin
               result:=(yDest[robot])*CELLSCALE-yCam;
          end;
       4: begin
               result:=(xDest[robot])*CELLSCALE-xCam;
          end;
       6: begin
               result:=yCam-(yDest[robot])*CELLSCALE;
          end;
       end;
   end
   else if followCircle[robot] = true then begin
        result := sqrt(sqr(xCam-rotationCenter[robot].x) + sqr(yCam-rotationCenter[robot].y))-CELLSCALE;
   end;

end;

function AngleToReference(robot:integer;xCam:double;yCam:double;thetaCam:double):double;
var
    r_x,r_y: double;
    alpha: double;
    produtointerno_rs,norma_r,norma_s: double;
begin
    if followLine[robot] = true then begin
        result:=DiffAngle(thetaDest[robot],thetaCam);
    end
    else if followCircle[robot] = true then begin
        r_x:=xCam-rotationCenter[robot].x;
        r_y:=yCam-rotationCenter[robot].y;

        //ângulo entre o vetor r e o vetor s (calculado antes do movimento circular)
        produtointerno_rs := r_x*s[robot].x + r_y*s[robot].y;
        norma_r := sqrt(sqr(r_x) + sqr(r_y));
        norma_s := sqrt(sqr(s[robot].x) + sqr(s[robot].y));
        alpha := arccos(produtointerno_rs / (norma_r*norma_s));

        result := DiffAngle(DiffAngle(thetaCam,thetaDest[robot]),(pi/2-alpha));
        if (result > (pi-THRESHOLD_ANGLE)) then begin
            result:=result-pi;
        end
        else if (result < (-pi+THRESHOLD_ANGLE)) then begin
            result:=result+pi;
        end;
    end;

end;

function Signal(value:integer):integer;
begin
     if value > 0 then begin
         result:=1;
     end
     else if value < 0 then begin
         result:=-1;
     end;
end;

procedure UpdateThetaDest(robot:integer;thetaCam:double);
begin
   if ((thetaDest[robot]-thetaCam < 0))
   then begin
       thetaDest[robot]:=thetaDest[robot]-pi*0.25;
   end
   else if ((thetaDest[robot]-thetaCam > 0))
   then begin
       thetaDest[robot]:=thetaDest[robot]+pi*0.25;
   end;
end;

procedure UpdateThetaDestPi(robot:integer;thetaCam:double);
begin
    if thetaDest[robot] >= 0 then begin
        thetaDest[robot]:=thetaDest[robot] - pi;
    end
    else if thetaDest[robot] < 0 then begin
        thetaDest[robot]:=thetaDest[robot] + pi;
    end;
end;

procedure UpdateThetaDestToMoveBack(robot:integer;thetaCam:double);
begin
     if ((thetaDest[robot]-thetaCam) > (pi + THRESHOLD_ANGLE))
     then begin
         thetaDest[robot]:=thetaDest[robot]-pi*2;
     end
     else if ((thetaDest[robot]-thetaCam) < (-pi - THRESHOLD_ANGLE))
     then begin
         thetaDest[robot]:=thetaDest[robot]+pi*2;
     end;

     if ((thetaDest[robot]-thetaCam < 0))
     then begin
         thetaDest[robot]:=thetaDest[robot]+pi*0.25;
     end
     else if ((thetaDest[robot]-thetaCam > 0))
     then begin
         thetaDest[robot]:=thetaDest[robot]-pi*0.25;
     end;
     UpdateThetaDestPi(robot,thetaCam);
end;

procedure UpdateThetaDestAfterPiRotation(robot:integer;thetaCam:double);
begin
   if ((thetaDest[robot]-thetaCam < 0))
   then begin
       thetaDest[robot]:=thetaDest[robot]+pi*0.25-pi;
   end
   else if ((thetaDest[robot]-thetaCam > 0))
   then begin
       thetaDest[robot]:=thetaDest[robot]-pi*0.25+pi;
   end;
end;

procedure UnpackUDPmessage(var xCam:array of double;var yCam:array of double; var thetaCam:array of double; var id_rob:integer;data:string);
var
    i,j:integer;
    X_a,Y_a:Double;
    c_a:integer;
    xCamStr,id_robstr,coms_countstr,yCamStr,thetaCamStr:string;
begin
    j:=2;
    if (data<>'') then begin
      if (data[j]<>'F') then begin
           while (data[j]<>'C') do begin
               id_robstr := id_robstr + data[j];
               j:=j+1;
            end;
            j:=j+1;
            while (data[j]<>'X') do begin
               coms_countstr := coms_countstr + data[j];
               j:=j+1;
            end;
            j:=j+1;
            while (data[j]<>'Y') do begin
               xCamStr := xCamStr + data[j];
               j:=j+1;
            end;

            j:=j+1;
            while (data[j]<>'T') do begin
               yCamStr := yCamStr + data[j];
               j:=j+1;
            end;

            j:=j+1;
            while ((data[j]<>'F')) do begin
               thetaCamStr := thetaCamStr + data[j];
               j:=j+1;
            end;

            X_a:=StrToFloat(xCamStr);
            Y_a:=StrToFloat(yCamStr);
            if ((form2.coms_flaws=1) and (form2.X_c_min<=X_a) and (form2.X_c_max>=X_a) and  (form2.Y_c_min<=Y_a) and (form2.Y_c_max>=Y_a)) then
            begin
            id_rob:= Strtoint(id_robstr);
            a_c_flaw[id_rob-1]:=1;
            xCam[id_rob-1] := 9999999;
            yCam[id_rob-1] := 9999999;
            thetaCam[id_rob-1] := 9999999;
            end else begin
            id_rob:= Strtoint(id_robstr);
            c_a:=Strtoint(coms_countstr);
            xCam[id_rob-1] := StrToFloat(xCamStr);
            yCam[id_rob-1] := StrToFloat(yCamStr);
            thetaCam[id_rob-1] := StrToFloat(thetaCamStr);
            a_c_flaw[id_rob-1]:=0;
            if c_a=999 then begin
              coms_count[id_rob-1]:=0;
            end else begin
               coms_count[id_rob-1]:=c_a;
            end;
            end;
            xCamStr:='';
            yCamStr:='';
            thetaCamStr:='';
            id_robstr:='';
            coms_countstr:='';
         end;
      end;
end;

procedure UpdateInitialPoints(var xCam:array of double;var yCam:array of double; var thetaCam:array of double; var id_rob:integer);
var
    i:integer;
begin
        i:=id_rob-1;
        form1.robots[i].pos_X := xCam[i];
        form1.robots[i].pos_Y:= yCam[i];
        if init[i]=0 then
        begin
        form1.robots[i].inicial_node:=get_closest_node_id(form1.full_nodelist,form1.robots[i].pos_X ,form1.robots[i].pos_Y,1);
        init[i]:=checkfornode(form1.full_nodelist, form1.robots[i].pos_X ,form1.robots[i].pos_Y,1);
        end
        else
        begin
        form1.robots[i].inicial_node:=updaterobotnode(form1.map,form1.full_nodelist,form1.robots[i].pos_X ,form1.robots[i].pos_Y,1,i);
        end;
        if ((thetaCam[i] <= pi/2 + THRESHOLD_ANGLE) and (thetaCam[i] >= pi/2 - THRESHOLD_ANGLE)) then begin
             form1.robots[i].Direction := 0;
        end
        else if ((thetaCam[i] <= pi/4 + THRESHOLD_ANGLE) and (thetaCam[i] >= pi/4 - THRESHOLD_ANGLE)) then begin
              form1.robots[i].Direction:= 1;
        end
        else if ((thetaCam[i] <= 0 + THRESHOLD_ANGLE) and (thetaCam[i] >= 0 - THRESHOLD_ANGLE)) then begin
              form1.robots[i].Direction:= 2;
        end
        else if ((thetaCam[i] <= -pi/4 + THRESHOLD_ANGLE) and (thetaCam[i] >= -pi/4 - THRESHOLD_ANGLE)) then begin
             form1.robots[i].Direction := 3;
        end
        else if ((thetaCam[i] <= -pi/2 + THRESHOLD_ANGLE) and (thetaCam[i] >= -pi/2 - THRESHOLD_ANGLE)) then begin
              form1.robots[i].Direction:= 4;
        end
        else if ((thetaCam[i] <= -pi*1.25 + THRESHOLD_ANGLE) and (thetaCam[i] >= -pi*1.25 - THRESHOLD_ANGLE)) then begin
              form1.robots[i].Direction := 5;
        end
        else if (((thetaCam[i] <= pi + THRESHOLD_ANGLE) and (thetaCam[i] >= pi - THRESHOLD_ANGLE)) or
                 ((thetaCam[i] <= -pi + THRESHOLD_ANGLE) and (thetaCam[i] >= -pi - THRESHOLD_ANGLE)))
        then begin
              form1.robots[i].Direction := 6;
        end
        else if ((thetaCam[i] <= pi*1.25 + THRESHOLD_ANGLE) and (thetaCam[i] >= pi*1.25 - THRESHOLD_ANGLE)) then begin
             form1.robots[i].Direction := 7;
        end;
end;



procedure UpdateSubmissions(var agvs:R_NODE;i:integer);
begin
    if ((agvs[i].inicial_node <> agvs[i].SubMissions[agvs[i].NumberSubMissions-1]))
    then begin

        if ((agvs[i].inicial_node = agvs[i].SubMissions[agvs[i].ActualSubMission-1]) and
            (agvs[i].ActualSubMission < agvs[i].NumberSubMissions))
        then begin
            agvs[i].ActualSubMission := agvs[i].ActualSubMission + 1;
            agvs[i].CounterSubMissions := agvs[i].CounterSubMissions + 1;
        end;
         agvs[i].onrest:=0;
    end else begin
     agvs[i].onrest:=1;
    end;
end;

{ TFControlo }

procedure TFControlo.udpComReceive(aSocket: TLSocket);
var
    data : string;
    i : integer;
    xCam,yCam,thetaCam: array[0..NUMBER_ROBOTS-1] of double;
    s1,s4,n_curr,aux1,idf:integer;
begin
    t1:=GetTickCount64();
    udpCom.GetMessage(data);

    if data <> '' then begin
      Edit1.Text:= data;
      Edit1.Color:= clgreen;
    end else begin
      Edit1.Text:= 'Erro';
      Edit1.Color:= clred;
    end;

    if data <> '' then begin
       Edit2.Text:=data[1];
    end;


    Edit8.Text:=FloatToStr(linearVelocities[0]);
    Edit9.Text:=FloatToStr(linearVelocities[1]);


    if (data <> '') then begin
        if ((flagVelocities = true) and (data[1] = 'R')) then begin
          Edit2.Text:='Hello';

          UnpackUDPmessage(xCam,yCam,thetaCam,id_rob,data);

          if ((xCam[id_rob-1]<>9999999) and (yCam[id_rob-1]<>9999999) and (thetaCam[id_rob-1]<>9999999)) then begin
          UpdateInitialPoints(xCam,yCam,thetaCam,id_rob);
          timestamp_coms[id_rob-1]:=total_seconds;
          if robots_flaws[id_rob-1].isactive=1 then begin
          s4:=length(form1.full_nodelist);
          idf:=0;
          for aux1:=0 to s4-1 do
           begin
           n_curr:=form1.full_nodelist[aux1].id;
           if n_curr=form1.robots[id_rob-1].inicial_node then
           begin
           idf:=n_curr;
            end;
           end;
           if idf=0 then begin
           aux_exit_flaw_node:=get_linked_node(form1.full_nodelist,agvs[i].pos_X,agvs[i].pos_y,1,CaminhosAgvs[i].coords[1].node);
           end else begin
           aux_exit_flaw_node:=idf;
           end;
           if robots_flaws[id_rob-1].isdetecting=1 then begin
              robots_flaws[id_rob-1].isdetecting:=0;
                robots_flaws[id_rob-1]:=save_exit_node(form1.robots,id_rob-1);
                //robots_flaws[id_rob-1].detected_nodes:=update_flaw_path(form1.robots,id_rob-1,CaminhosAgvs);
             end else begin
           if robots_flaws[id_rob-1].out_node[robots_flaws[id_rob-1].flaw_ind]=aux_exit_flaw_node then
                begin

                end else begin
                  robots_flaws[id_rob-1].detected_nodes[robots_flaws[id_rob-1].flaw_ind]:=update_afected_nodes( robots_flaws[id_rob-1].out_node[robots_flaws[id_rob-1].flaw_ind],robots_flaws[id_rob-1].in_node[robots_flaws[id_rob-1].flaw_ind],id_rob-1,CaminhosAgvs,robots_flaws[id_rob-1]);
                end;
                end;

                robots_flaws[id_rob-1].isactive:=0;
          end;
          end;
          i:=0;
          while i<NUMBER_ROBOTS do begin
              if ((type_of_movement[i]=0)) then begin
                 Edit4.Text:='TEA_Init';
                  followLine[i]:=false;
                  followCircle[i]:=false;
                  rotate[i]:=false;

                  UpdateSubmissions(form1.robots,i);

                  TEArun(form1.map,form1.robots,CaminhosAgvs);

                  //Deteta troca de prioridades e sai do ciclo para que não exista mistura de informação em índices errados
                  if ((flagChange = true) and (totalTrocas < MAX_EXCHANGES) and (totalTrocas <> totalValidations)) then begin
                    flagChange:=false;
                    totalTrocas:=0;
                    totalValidations:=0;
                    break;
                  end
                  else begin
                    trocas:=0;
                    totalTrocas:=0;
                  end;
                  step_complete:=check_step_comp(CaminhosAgvs,form1.robots,current_step);
                  //timestamp_coms[i]:=total_seconds;
              end else if ((timestamp_coms[i]+2<=total_seconds) and (type_of_movement[i]<>0) and (timestamp_coms[i]<>0))  then begin
              //coms flaw no communication with robot
              if robots_flaws[i].isactive=0 then begin
               robots_flaws[i]:=save_flaw_location(CaminhosAgvs,form1.robots,i,form1.full_nodelist,current_step[i]);
               end else begin
                robots_flaws[i].active_consecutive_hits:= robots_flaws[i].active_consecutive_hits+1;
               end;
               timestamp_coms[i]:=total_seconds;
              end else begin
              step_complete:=check_step_comp(CaminhosAgvs,form1.robots,current_step);
              if i=0 then begin
              Edit6.Text:=inttostr(current_step[i]);
              end else if i=1 then begin
              Edit7.Text:=inttostr(current_step[i]);
              end else if i=2 then begin
              Edit8.Text:=inttostr(current_step[i]);
              end else if i=3 then begin
              Edit9.Text:=inttostr(current_step[i]);
              end;
              {if ((step_complete[i]=0) or  (type_of_movement[i]=1)) then begin
                  Edit4.Text:='MOVE';
              end}
               if check_if_still_on_path(CaminhosAgvs,Form1.robots,i,0.05,current_step)=1 then begin
                 // if check_if_still_on_path(CaminhosAgvs,Form1.robots,i,0.05,current_step)=1 then
                 // begin
                  b_pathsend:=1;  //blocks the sending of a error filled path mid planning

                  Edit4.Text:='TEA';

                  UpdateSubmissions(form1.robots,i);

                  TEArun(form1.map,form1.robots,CaminhosAgvs);
                  //Deteta troca de prioridades e sai do ciclo para que não exista mistura de informação em índices errados
                  if ((flagChange = true) and (totalTrocas < MAX_EXCHANGES) and (totalTrocas <> totalValidations)) then begin
                    flagChange:=false;
                    totalTrocas:=0;
                    totalValidations:=0;
                    break;
                  end
                  else begin
                    trocas:=0;
                    totalTrocas:=0;
                  end;
                  step_complete:=check_step_comp(CaminhosAgvs,form1.robots,current_step);
                  b_pathsend:=0;
                 { end else if check_if_still_on_path(CaminhosAgvs,Form1.robots,i,0.05,current_step)=0 then
                  begin
                     Edit4.Text:='Next_Step';
                     //current_step[i]:=current_step[i]+1;
                 end;}
              end else begin
                    if ((step_complete[i]=0) or  (type_of_movement[i]=1)) then begin
                        Edit4.Text:='MOVE';
                     end else begin
                         Edit4.Text:='Next_Step';
                         current_step[i]:=current_step[i]+1;
                     end;

              end;
              end;
              i:=i+1;
          end;
        end;
    end;


    if data = 'MIP1' then begin
        flagMessageInitialPositions:=false;
        //flagVelocities:=true;
        Edit3.Text:='MIP1';
    end;
    t2:=GetTickCount64();
    tick_p:=t2-t1;
    Edit10.Text:=IntToStr(tick_p);
end;
procedure TFControlo.TimerSendTimer(Sender: TObject);
var

    l1:integer;
    aux1:integer;
    count:integer;
    messagelist:Tstringlist;
begin
    if flagMessageInitialPositions = true then begin
       udpCom.SendMessage(MessageInitialPositions, '127.0.0.1:9808');
       ind_robo:=0;
    end;

    if ((flagVelocities = true) and (b_pathsend<>1)) then begin

        if ind_robo<NUMBER_ROBOTS then begin
           if a_c_flaw[ind_robo]=0 then begin
           messagelist:=TStringList.Create;
           Ca[ind_robo]:=checkforpathcompletion(form1.robots,ind_robo);
           messagelist.Add('N' + IntToStr( form1.robots[ind_robo].id_robot));
           messagelist.Add('P' + IntToStr( form1.robots[ind_robo].InitialIdPriority));
           messagelist.Add('T' + IntToStr( Ca[ind_robo]));
           messagelist.Add('S' + IntToStr(get_steps(CaminhosAgvs,ind_robo)-1));
           l1:=length((CaminhosAgvs[ind_robo].coords));
           count:=0;
           for aux1:=current_step[ind_robo] to l1-1 do begin
           if ((getXcoord(CaminhosAgvs[ind_robo].coords[aux1].node)<3) and (getYcoord(CaminhosAgvs[ind_robo].coords[aux1].node)<3)and (count<8)) then
           begin
           messagelist.Add('I' + IntToStr(aux1));
           messagelist.Add('X' + FloatToStr(round2(getXcoord(CaminhosAgvs[ind_robo].coords[aux1].node),3)));
           messagelist.Add('Y' + FloatToStr(round2(getYcoord(CaminhosAgvs[ind_robo].coords[aux1].node),3)));
           messagelist.Add('D' + IntToStr(CaminhosAgvs[ind_robo].coords[aux1].direction));
           count:=count+1;
           end;
           end;
           messagelist.Add('F');
           MessageVelocities:=messagelist.Text;
           if count>1 then
           begin
           udpCom.SendMessage(MessageVelocities, '127.0.0.1:9808');
           end;
           messagelist.free;
           ind_robo:=ind_robo+1;
           end else begin
            ind_robo:=ind_robo+1;
           end;
        end else begin
        ind_robo:=0;
        end;
        Edit5.Text:=inttostr(form1.robots[0].Direction);
        Edit6.Text:=inttostr(form1.robots[3].Direction);
        //Edit7.Text:=MessageVelocities1;
        MessageVelocities1:='';
        MessageVelocities:='';
        ct:=1;
        s_time:=s_time+80;
        if s_time>=1000 then begin
         total_seconds:=total_seconds+1;
         s_time:=0;
    end;
    end;

end;

procedure TFControlo.FormShow(Sender: TObject);
var
    i:integer;
begin
    Edit1.Text:= 'Error';
    Edit1.Color:= clred;

    udpCom.Disconnect;
    if udpCom.Connect('127.0.0.1', 4040) then begin
        Edit1.Text:= 'connection open';
        Edit1.Color:= clyellow;
    end;
    if udpCom.Listen(4040) then begin
        Edit1.Text:= 'open ports';
    end;

    flagMessageInitialPositions:=true;
    flagVelocities:=false;
    MessageVelocities:='';
    contador:=0;
    trocas:=0;
    flagChange := false;
    totalTrocas := 0;
    totalValidations := 0;
    ind_robo:=0;
    b_pathsend:=0;
    InitialPointsForAllRobots(form1.robots);


    i:=0;
    while i<NUMBER_ROBOTS do begin
      linearVelocities[i]:=0;
      angularVelocities[i]:=0;
      i:=i+1;
    end;
   ct:=0;
end;

procedure TFControlo.SendButtonClick(Sender: TObject);
var
    i:integer;
begin
    flagVelocities:=true;
    s_time:=0;
    total_seconds:=0;
    for i:=0 to NUMBER_ROBOTS-1 do
    begin
    timestamp_coms[i]:=0;
    type_of_movement[i]:=0;
    current_step[i]:=1;
    ghost_nodes[i]:=0;
    robots_flaws[i].isactive:=0;
    robots_flaws[i].isdetecting:=0;
    robots_flaws[i].active_consecutive_hits:=0;
    robots_flaws[i].n_flaws:=0;
    end;

end;

procedure TFControlo.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
    udpCom.Disconnect();
end;

procedure TFControlo.FormCreate(Sender: TObject);
var
    aux1:integer;
begin
  for aux1:=0 to NUMBER_ROBOTS-1 do begin
  pre_coms_count[aux1]:=10000
  end;
end;

{procedure TFControlo.Coms_timeout_timerTimer(Sender: TObject);
 var
     aux1,aux2:integer;
     id1_aux,id2_aux:integer;
begin
{  for aux1:=0 to NUMBER_ROBOTS-1 do begin
      if pre_coms_count[aux1]=coms_count[aux1] then begin
         robots_flaws[aux1].isactive:=1;
         aux2:=robots_flaws[aux1].active_consecutive_hits;
         robots_flaws[aux1].active_consecutive_hits:=aux2+1;
         robots_flaws[aux1].in_node:=CaminhosAgvs[aux1].coords[1].node;
      end else begin
           pre_coms_count[aux1]:=coms_count[aux1];
           robots_flaws[aux1].active_consecutive_hits:=0;
           if robots_flaws[aux1].isactive=1 then begin
              id1_aux:=get_closest_node_id(form1.full_nodelist,form1.robots[aux1].pos_X,form1.robots[aux1].pos_Y,1);
              id2_aux:=get_linked_node(form1.full_nodelist,form1.robots[aux1].pos_X,form1.robots[aux1].pos_Y,1,id1_aux);
              if id2_aux=robots_flaws[aux1].in_node then begin
              robots_flaws[aux1].out_node:=id2_aux;
              end else begin
                robots_flaws[aux1].out_node:=id1_aux;
              end;
              robots_flaws[aux1].isactive:=0;
           end;
      end;
  end;}
end; }

end.
