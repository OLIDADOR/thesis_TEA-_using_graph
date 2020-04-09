unit TEAstar;

{$mode objfpc}{$H+}


//  FUNCIONAMENTO
//      Necessário: - Atualização da posição dos robôs a cada instante de
//                    planeamento (proveniente do sistema de localização baseado
//                    em visão)
//                  - Tarefas (ponto inicial e final)
//                  - Lista de Prioridades das tarefas

//      Processo:   - Implementa-se o algoritmo em todos os robôs, sendo que
//                    cada um fará, a cada instante, o planeamento completo de
//                    todos os robôs de modo a conhecer o modo de atuar dos
//                    robôs com prioridade inferior à sua (ou seja, conhecer
//                    os trajetos já colocados como obstáculos); assim apenas
//                    tem de saber qual a prioridade a que lhe corresponde e
//                    mover-se mediante o que foi para si planeado

//      Desvantagem: - Maior processamento a cada instante

//      Alternativa: - Haver possibilidade de cada robô aceder a uma base de
//                     dados comum onde é guardado um mapa com o estado de todas
//                     as células até ao momento; assim conhecia os caminhos
//                     já planeados pelos robôs de prioridade inferior e
//                     enviava também o seu para a BD após acabar o planeamento


interface

uses
  Classes, SysUtils, Graphics, laz2_XMLRead, laz2_DOM, Forms, math;

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





   //link_telement = object
   //private
   //  {private declarations}
   //public
   //  {public declarations}
   //  var
   //  id_l: array of integer;
   //  id_nodes: array of integer;
   //  distance: double;
   //end;
   //link_table = object
   //private
   //  {private declarations}
   //public
   //  {public declarations}
   //  var
   //  table:array of array link_telement;
   //end;


    Robot_Pos_info = object
   private
     {private declarations}
   public
     {public declarations}
     var
     id_robot:integer;
     current_nodes:array of integer;
     inicial_node:integer;
     target_node:integer;
     target_node_step:integer;
     inicial_partial_node:integer;
     pos_X:Double;
     pos_Y:Double;
     Direction:Double;
     ipos_X:Double;
     ipos_Y:Double;
     iDirection:Double;
     SubMissions: array[0..MAX_SUBMISSIONS] of integer;
     NumberSubMissions: integer;
     CounterSubMissions: integer;
     ActualSubMission: integer;
     InitialIdPriority: integer;
   end;

   TEA_Graph_node = object
   private
     {private declarations}
   public
     {public declarations}
     var
     id:integer;
     pos_X:Double;
     pos_Y:Double;
     H:double;
     G:double;
     Parent_node:integer;
     Parent_step:integer;
     //status:integer;
     HeapIdx:integer;
     direction:double;
     links:array of link_full;
   end;
PAStarCell = ^TEA_Graph_node;  // PAStarCell is a pointer to TAStarCell, just points to an adress
 TAStarHeapArray = record
      data: array of PAStarCell;
      count: integer;
    end;

  TAStarProfiler = record
    RemovePointFromAStarList_count: integer;
    RemoveFromOpenList_count: integer;
    AddToOpenList_count: integer;
    HeapArrayTotal: integer;
    comparesTotal: integer;
    iter: integer;
  end;

  TAStarMap = record
    TEA_GRAPH: array of array of TEA_Graph_node;
    GraphState: array of array of byte;
    HeapArray: TAStarHeapArray;
    Profiler: TAStarProfiler;
  end;


       r_node=array of Robot_Pos_info;

      a_node=array of node_full;





  ////TGridCoord = record
  ////  x, y, t_step, direction: Smallint;
  ////end;


  //TAStarCell = record
  //  G,H: double;
  //  ParentPoint: TGridCoord;
  //  HeapIdx: integer;
  //  MyCoord: TGridCoord;
  //end;

  //TAStarGridState = array[0..AStarGridXSize, 0..AStarGridYSize, 0..NUM_LAYERS] of byte;
  //TAStarGrid = array[0..AStarGridXSize, 0..AStarGridYSize, 0..NUM_LAYERS-1] of TAStarCell;
  //
  //  TAStarHeapArray = record
  //    data: array[0 .. AStarHeapArraySize-1] of PAStarCell;
  //    count: integer;
  //  end;
  //
  //TAStarProfiler = record
  //  RemovePointFromAStarList_count: integer;
  //  RemoveFromOpenList_count: integer;
  //  AddToOpenList_count: integer;
  //  HeapArrayTotal: integer;
  //  comparesTotal: integer;
  //  iter: integer;
  //end;
  //
  //TAStarMap = record
  //  Grid: TAStarGrid;
  //  GridState: TAStarGridState;
  //  HeapArray: TAStarHeapArray;
  //  Profiler: TAStarProfiler;
  //end;
  //
  //Robot = record
  //  InitialPoint, TargetPoint, InitialPointParcial: TGridCoord;
  //  Color: string;
  //  SubMissions: array[0..MAX_SUBMISSIONS] of TGridCoord;
  //  NumberSubMissions: integer;
  //  CounterSubMissions: integer;
  //  ActualSubMission: integer;
  //  InitialIdPriority: integer;
  //end;
  //
  //RobotsTeam = array[0..NUMBER_ROBOTS-1] of Robot;
  //

  nodes= record
    node:integer;
    steps:integer;
    direction:integer;
  end;

  Caminho = record
      coords: array[0 .. NUM_LAYERS] of nodes;
      steps: integer;
  end;
  //
  Caminhos = array[0..NUMBER_ROBOTS-1] of Caminho;


var
    noPath: boolean;
    flagTargetOverlap: boolean;
    trocas: integer;
    flagTargetOverlapInverse: boolean;
    errorMessage: integer;
    vehicleError: integer;

procedure A_starTimeInit(var Map: TAStarMap; agv: Robot_Pos_info);
procedure A_starTimeInitSubMission(var Map: TAStarMap; agv: Robot_Pos_info);
function A_starTimeGo(var Map: TAStarMap; var CaminhosAgvs: Caminhos; var agvs: r_node; maxIter: integer):integer;
function get_node_dir (var Map:TAStarMap;n1:integer;n2:integer):integer;
function TEAstep(var Map:TAStarMap; var agv:Robot_Pos_info; var steps:integer; var curPnt:integer; var curPnt_t_step:integer):double;
function get_int_dirr( var direction:Double):integer;
procedure StorePath (var Map:TAStarMap; var agvs:r_node; var CaminhosAgvs:Caminhos; vehicle:integer; steps:integer);
procedure CloserNeighborhoodAsObstacle (var Map:TAStarMap; i:integer; tstep:integer);
procedure LargeNeighborhoodAsObstacle (var Map:TAStarMap; i:integer; j:integer; tstep:integer);
procedure UpdatePathDirections (var CaminhosAgvs:Caminhos; vehicle:integer;  map:TAStarMap);
procedure FreeCellsToVirgin (var Map:TAStarMap);
procedure CellsToVirgin(var Map:TAStarMap;startLayer:integer);
procedure PathsToVirgin (var Map:TAStarMap);
procedure CleanHeapArray (var Map:TAStarMap);
procedure InitialPositionAsObstacles (var vehicle: integer; Map: TAStarMap; agvs: r_node);
function CalcH(var Map: TAStarMap; Pi, Pf: integer): double; inline;
function CalcF(var Map: TAStarMap; idx: integer): double; inline;
procedure InsertInOpenList( var Map: TAStarMap; id: integer; step:integer);
procedure UpdateHeapPositionByPromotion(var Map: TAStarMap; idx: integer); inline;
procedure SwapHeapElements(var Map: TAStarMap; idx1, idx2: integer); inline;
procedure RemoveFromOpenList(var Map: TAStarMap; out Pnt: integer); inline;
procedure UpdateHeapPositionByDemotion(var Map: TAStarMap; idx: integer); inline;


implementation
uses
  unit1;
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure A_starTimeInit(var Map: TAStarMap; agv: Robot_Pos_info);
begin
  //update the initial direction in order to be considered when the initial point
  //is inserted on heapArray
  Map.TEA_GRAPH[agv.inicial_node-1][0].direction:=agv.Direction;

  InsertInOpenList(Map, agv.inicial_node,0);
  Map.TEA_GRAPH[agv.inicial_node-1][0].H := CalcH( Map, agv.inicial_node, agv.target_node);
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure A_starTimeInitSubMission(var Map: TAStarMap; agv: Robot_Pos_info);
begin
  //update the initial direction in order to be considered when the initial point
  //is inserted on heapArray
  Map.TEA_GRAPH[agv.inicial_partial_node-1][0].direction:=agv.Direction;

   InsertInOpenList(Map, agv.inicial_partial_node,0);
   Map.TEA_GRAPH[agv.inicial_partial_node-1][0].H := CalcH( Map, agv.inicial_partial_node, agv.target_node);
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

function A_starTimeGo(var Map: TAStarMap; var CaminhosAgvs:Caminhos; var agvs: r_node; maxIter: integer):integer;
var
    curPnt: integer;
    curPnt_t_step: integer;
    count,vehicle,steps: integer;

begin

  flagTargetOverlap:=false;
  noPath:=false;  //false: there is path / true: not
  errorMessage:=0;
  vehicleError:=0;

  for count := 0 to (length(agvs)-1) do begin

      curPnt:= 0;
      curPnt_t_step := 0;
      steps := 0;

      A_starTimeInit(Map,agvs[count]);

      agvs[count].target_node := agvs[count].SubMissions[agvs[count].ActualSubMission-1];

      //The initial position of each agv can´t be considered by the others as possible path in k={0,1}
      //independently of its priorities
      vehicle:=count;
      InitialPositionAsObstacles(vehicle,Map,agvs);

      result:=vehicle;

      while true do begin

        if ((maxIter > 0) and (Map.Profiler.iter >= maxIter)) then begin
          noPath:=true;
          errorMessage:=1;
          vehicleError:=count;
          break;
        end;

        if TEAstep(Map,agvs[count],steps,curPnt,curPnt_t_step) > 0 then break;

        // found the way
        if Map.GraphState[agvs[count].target_node-1][agvs[count].target_node_step] = CLOSED then begin
          if agvs[count].CounterSubMissions = agvs[count].NumberSubMissions then begin
            agvs[count].CounterSubMissions := agvs[count].ActualSubMission;
            break;
          end
          else begin
             agvs[count].CounterSubMissions := agvs[count].CounterSubMissions +1;
             agvs[count].inicial_partial_node := agvs[count].target_node;
             agvs[count].target_node := agvs[count].SubMissions[agvs[count].CounterSubMissions-1];
             CleanHeapArray(Map);
             CellsToVirgin(Map,agvs[count].target_node_step);
             A_starTimeInitSubMission(Map,agvs[count]);
          end;
        end;

        // there is no path
        if Map.HeapArray.count = 0 then begin
          noPath:=true;
          errorMessage:=2;
          vehicleError:=count;
          break;
        end;

      end;

      //Clean the HeapArray (Open Nodes Array)
      CleanHeapArray(Map);

      //Stores the path of the robot and mark it as obstacle as well as it
      //neighbourhood in every layer, if path is found
      if (noPath=false) then begin
         StorePath(Map,agvs,CaminhosAgvs,vehicle,steps);
      end;

      //Changes the state of the non obstacles cells to VIRGIN in every layer
      FreeCellsToVirgin(Map);

      //if path is not found, returns the vehicle and do not plan the next robots
      if ((noPath=true) or (flagTargetOverlap=true)) then break;

      vehicle:=vehicle+1;

  end;

  PathsToVirgin(Map);

  //if everything works great
  if ((noPath=false) and (flagTargetOverlap=false)) then result:=-1;

end;
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
function get_node_dir (var Map:TAStarMap;n1:integer;n2:integer):integer;
var
 x1,x2,y1,y2:Double;
 dist:double;
 d_x,d_y,aux2,aux3,aux1:double;
 angle:double;
begin
  x1:=Map.TEA_GRAPH[n1-1][0].pos_X;
  x2:=Map.TEA_GRAPH[n2-1][0].pos_X;
  y1:=Map.TEA_GRAPH[n1-1][0].pos_Y;
  y2:=Map.TEA_GRAPH[n2-1][0].pos_Y;
  d_x:=x2-x1;
  d_y:=y2-y1;
  aux2:=d_x*d_x;
  aux3:=d_y*d_y;
  aux1:=aux2+aux3;
  dist:=sqrt(aux1);
  angle:=arccos(dist/x2);
  get_node_dir:=get_int_dirr(angle);
end;
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

function TEAstep(var Map:TAStarMap; var agv:Robot_Pos_info; var steps:integer; var curPnt:integer; var curPnt_t_step:integer):double;
var
    newPnt: integer;
    newPnt_step:integer;
    newPnt_dirr:real;
    newG,auxCost: double;
    edge: integer;
    i,j,l1: integer;
    contador: integer;
    n1,n2:integer;
    curr_dirr,node_dirr,rot:Double;
    rot_abs:double;
begin

   contador:=0;

   //VERIFICAR RESULT NO PROCEDURE "GO"
   result := 0;

   //"inc" increase the adress of the pointer (with syze of variable type)
   inc(Map.Profiler.iter);
   inc(Map.Profiler.HeapArrayTotal, Map.HeapArray.count);

   //Verify if the curPnt is the target point
   if (curPnt= agv.target_node) then begin
      Map.TEA_GRAPH[curPnt-1][curPnt_t_step].H := CalcH(Map, curPnt, agv.target_node);
      result := Map.TEA_GRAPH[curPnt-1][curPnt_t_step].H;
      //if ((agv.TargetPoint.x = agv.InitialPoint.x) and (agv.TargetPoint.x = agv.InitialPoint.x)) then begin
      //   result:=0;
      //end;
      agv.target_node_step:=curPnt_t_step;
   end

   else if curPnt_t_step>= NUM_LAYERS then begin
      //writeln('More Layers Required');
      result:=1;
   end

   else begin

      //Take from the open list the node with the lowest f
      RemoveFromOpenList(Map,curPnt);

      Map.GraphState[curPnt-1][curPnt_t_step] := CLOSED;

//
//      if((curPnt.x mod 2) = 0) and ((curPnt.y mod 2) = 0) then begin
//        //it's a center point of a cell
//        edge:=0;
//      end
//      else begin  //it's an edge
//       if((curPnt.x mod 2) = 0) then begin        //verify if it is an horizontal edge
//          edge:=1;
//       end
//       else begin                               //verify if it is a vertical edge
//          edge:=2;
//       end;
//      end;


   l1:=length(map.TEA_GRAPH[curPnt-1][curPnt_t_step].links);
   for i:=0 to l1 do
   begin
      //Gerar a vizinhança
      //Verificar a rotação
      if i=0 then
      begin
        //Possibilidade de permanecer no mesmo ponto
        newPnt:=map.TEA_GRAPH[curPnt-1][curPnt_t_step].id;
        newPnt_step:= curPnt_t_step+1;
      end
      else
      begin
      n1:=map.TEA_GRAPH[curPnt-1][curPnt_t_step].id;
      n2:=map.TEA_GRAPH[curPnt-1][curPnt_t_step].links[i-1].node_to_link;
      curr_dirr:=agv.Direction;
      node_dirr:=get_node_dir(Map,n1,n2);
      rot:=node_dirr-curr_dirr;
      rot_abs:=abs(rot);
      //move parallel or diagonaly = 1 step
      //rotate 90º and move = 2 steps
      //rotate 45º and move parallel = 1 step
      //rotate 45º and move diagonally = 2 steps
      newPnt:=n2;
      if (rot_abs<2) then
      begin
        newPnt_step:= curPnt_t_step+1;
      end
      else
      begin
          newPnt_step:= curPnt_t_step+2;
      end;
       newPnt_dirr:=node_dirr;
      end;

      //for i:=-1 to 1 do
      //begin
      //
      //    for j:=-1 to 1 do
      //    begin
      //
      //        //verify if the current point is inside the limits of the map
      //        //(>=2 because 1 corresponds to the wall)
      //        //edge = 1 and edge = 2 has neighbourhood 6 (i<>0 / j<>0 avoid two vertices)
      //        //edge = 0 has neighbourhood 4 (i=0 or j=0 avoid the four vertices)
      //
      //        if((curPnt.x + i >= 2) and (curPnt.y + j >= 2) and
      //           (curPnt.x + i < AStarGridXSize) and (curPnt.y + j < AStarGridYSize) and
      //           (((edge = 1) and (j<>0)) or ((edge = 2) and (i<>0)) or
      //           ((edge = 0) and ((j = 0) or (i = 0)))))
      //        then begin
      //
      //              //generates the neighborhood
      //              //move parallel or diagonaly = 1 step
      //              //rotate 90º and move = 2 steps
      //              //rotate 45º and move parallel = 1 step
      //              //rotate 45º and move diagonally = 2 steps
      //              newPnt.x := curPnt.x + i;
      //              newPnt.y := curPnt.y + j;
      //              case curPnt.direction of
      //                 0,4: begin
      //                          if ((i<>0) and (j<>0)) then begin newPnt.t_step := curPnt.t_step + 2; end
      //                          else if ((i<>0) and (j=0)) then begin newPnt.t_step := curPnt.t_step + 2; end
      //                          else if ((i=0) and (j<>0)) then begin newPnt.t_step := curPnt.t_step + 1; end
      //                          else if ((i=0) and (j=0)) then begin newPnt.t_step := curPnt.t_step + 1; end;
      //                      end;
      //                 2,6: begin
      //                          if ((i<>0) and (j<>0)) then begin newPnt.t_step := curPnt.t_step + 2; end
      //                          else if ((i<>0) and (j=0)) then begin newPnt.t_step := curPnt.t_step + 1; end
      //                          else if ((i=0) and (j<>0)) then begin newPnt.t_step := curPnt.t_step + 2; end
      //                          else if ((i=0) and (j=0)) then begin newPnt.t_step := curPnt.t_step + 1; end;
      //                      end;
      //                 1,5: begin
      //                          if (i=j) then begin newPnt.t_step := curPnt.t_step + 1; end
      //                          else if (i=-j) then begin newPnt.t_step := curPnt.t_step + 2; end
      //                          else if ((i=0) and (j<>0)) then begin newPnt.t_step := curPnt.t_step + 1; end
      //                          else if ((i<>0) and (j=0)) then begin newPnt.t_step := curPnt.t_step + 1; end
      //                          else if ((i=0) and (j=0)) then begin newPnt.t_step := curPnt.t_step + 1; end;
      //                      end;
      //                 3,7: begin
      //                          if (i=j) then begin newPnt.t_step := curPnt.t_step + 2; end
      //                          else if (i=-j) then begin newPnt.t_step := curPnt.t_step + 1; end
      //                          else if ((i=0) and (j<>0)) then begin newPnt.t_step := curPnt.t_step + 1; end
      //                          else if ((i<>0) and (j=0)) then begin newPnt.t_step := curPnt.t_step + 1; end
      //                          else if ((i=0) and (j=0)) then begin newPnt.t_step := curPnt.t_step + 1; end;
      //                      end;
      //              end;
      //
      //              //"rosa dos ventos": North (j direction) is equal to direction 0
      //              //counting clockwise
      //              if ((i=0) and (j=1)) then begin newPnt.direction:=0; end
      //              else if ((i=1) and (j=1)) then begin newPnt.direction:=1; end
      //              else if ((i=1) and (j=0)) then begin newPnt.direction:=2; end
      //              else if ((i=1) and (j=-1)) then begin newPnt.direction:=3; end
      //              else if ((i=0) and (j=-1)) then begin newPnt.direction:=4; end
      //              else if ((i=-1) and (j=-1)) then begin newPnt.direction:=5; end
      //              else if ((i=-1) and (j=0)) then begin newPnt.direction:=6; end
      //              else if ((i=-1) and (j=1)) then begin newPnt.direction:=7; end;


                    case Map.GraphState[newPnt-1][newPnt_step] of

                        OBSTACLEWALL: continue;

                        OBSTACLEROBOT: continue;

                        CLOSED: continue;

                        VIRGIN: begin
                            //The node is not in the Open List neither in the Close List
                            //So it is added to the Open List with properties updated

                            with Map.TEA_GRAPH[newPnt-1][newPnt_step] do begin
                              Parent_node := curPnt;
                              //ParentPoint.y := curPnt.y;
                              Parent_step := curPnt_t_step;
                              //ParentPoint.direction := curPnt.direction;
                              H := CalcH(Map, newPnt, agv.target_node);
                              //mycoord.x := newPnt.x;
                              //mycoord.y := newPnt.y;
                              //mycoord.t_step := newPnt.t_step;
                              //mycoord.direction := newPnt.direction;

                              //The robot stays in the same point in both time stamps
                              if (i=0) then begin
                                    G := Map.TEA_GRAPH[curPnt-1][curPnt_t_step].G + COST1;
                              end
                              else begin
                                    G := Map.TEA_GRAPH[curPnt-1][curPnt_t_step].G + COST2*abs(map.TEA_GRAPH[curPnt][curPnt_t_step].links[i-1].distance);
                              end;

                              //to reflect the rotation cost
                              //(cost of one more step stopped)
                              if (newPnt_step = curPnt_t_step + 2) then begin
                                    G := G + COST1;
                              end;

                            end;

                            //Add node successor to the Open List
                            InsertInOpenList(Map, newPnt,newPnt_step);

                        end;

                        OPENED: begin

                            if (i=0)  then begin
                                  auxCost := COST1;
                            end
                            else begin
                                  auxCost := COST2*abs(map.TEA_GRAPH[curPnt-1][curPnt_t_step].links[i-1].distance);
                            end;

                            if (newPnt_step = curPnt_t_step + 2) then begin
                                  auxCost := auxCost + COST1;
                            end;

                            //newG is the cumulative G cost from the current point to the successor
                            //so, sometimes it can be bigger than the real cost to the initial point
                            newG := Map.TEA_GRAPH[curPnt-1][curPnt_t_step].G + auxCost;

                            //if G(newPoint)<=newG nothing happens
                            //else updates G cost of the newPoint and updates its parent
                            //its parent become the node that was expanded
                            if newG < Map.TEA_GRAPH[curPnt-1][curPnt_t_step].G then begin
                              Map.TEA_GRAPH[newPnt-1][newPnt_step].G := newG;
                              Map.TEA_GRAPH[newPnt-1][newPnt_step].Parent_node := curPnt;
                              UpdateHeapPositionByPromotion(Map, Map.TEA_GRAPH[newPnt-1][newPnt_step].HeapIdx);
                            end;

                        end;
                    end;
          end;
      end;


   steps:=curPnt_t_step;

   contador:=contador+1;

end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
  function get_int_dirr( var direction:Double):integer;

  begin
    if (Direction=90) then begin get_int_dirr:=0; end
    else if (((Direction>0) and (Direction<90)) or ((Direction<-270) and (Direction>-360))) then begin get_int_dirr:=1; end
    else if (Direction=0) then begin get_int_dirr:=2; end
    else if (((Direction<0) and (Direction>-90)) or ((Direction>270) and (Direction<360))) then begin get_int_dirr:=3; end
    else if (Direction=-90) then begin get_int_dirr:=4; end
    else if (((Direction<-90) and (Direction>-180)) or ((Direction<270) and (Direction>180))) then begin get_int_dirr:=5; end
    else if (Direction=180) then begin get_int_dirr:=6;end
    else if (((Direction>-270) and (Direction<-180)) or ((Direction<180) and (Direction>90))) then begin get_int_dirr:=7; end;
  end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
procedure StorePath (var Map:TAStarMap; var agvs:r_node; var CaminhosAgvs:Caminhos; vehicle:integer; steps:integer);
var
    i,j,tstep:integer;
    aux_i,aux_j,aux_tstep:integer;
    count: integer;
    v: integer;
    d: integer;
begin

    //Store the path of the robot and mark it as obstacle as well as it
    //neighbourhood in every layer

    i:=agvs[vehicle].target_node;
    tstep:=agvs[vehicle].target_node_step;

    CaminhosAgvs[vehicle].coords[tstep].node:=i;
    CaminhosAgvs[vehicle].coords[tstep].steps:=tstep;
    CaminhosAgvs[vehicle].steps:=steps;


    //The target point is considered as obstacle in every layers, starting at the
    //step that reaches the goal until the maximum NUM_LAYERS
    //Only if it was detected maximum exchanges of priorities
    if ((trocas >= MAX_EXCHANGES) or (flagTargetOverlapInverse = true)) then begin
        count:=0;
        while steps+count < NUM_LAYERS do begin
            Map.GraphState[i-1][steps+count] := OBSTACLEROBOT;

            CloserNeighborhoodAsObstacle(Map,i,steps+count);
            //LargeNeighborhoodAsObstacle(Map,i,j,steps+count);

            count:=count+1;
        end;
    end;


    //count < NUM_LAYERS guarantees that the execution don't crash with an
    //infinite cycle when a robot closed the target, but it can´t plan a viable path
    //(therefore, variable "noPath" stays equals to false, so this function runs)
    count:=0;
    while (((i<>agvs[vehicle].inicial_node) or
           (tstep<>0)) and (count < NUM_LAYERS)) do begin

           aux_i:=i;
           aux_tstep:=tstep;

           i:=Map.TEA_GRAPH[aux_i-1][aux_tstep].Parent_node;
           tstep:=Map.TEA_GRAPH[aux_i-1][aux_tstep].Parent_step;

           //if there was a rotation and parent node dist two steps of the child
           //fill the step between them with the coords of tstep (stop node)
           if tstep = aux_tstep-2 then begin
               CaminhosAgvs[vehicle].coords[tstep+1].node:=i;
               CaminhosAgvs[vehicle].coords[tstep+1].steps:=tstep+1;
               Map.GraphState[i-1][tstep+1]:=OBSTACLEROBOT;

               CloserNeighborhoodAsObstacle(Map,i,tstep+1);
               //LargeNeighborhoodAsObstacle(Map,i,j,tstep+1);

           end;

           Map.GraphState[aux_i-1][aux_tstep]:=OBSTACLEROBOT;


           //it's necessary mark the neighbourhood of the current node as
           //obstacle, in the same temporal layer, in order to avoid
           //collision due to the robot's dimensions
           CloserNeighborhoodAsObstacle(Map,aux_i,aux_tstep);
           //LargeNeighborhoodAsObstacle(Map,aux_i,aux_j,aux_tstep);


           CaminhosAgvs[vehicle].coords[tstep].node:=i;
           CaminhosAgvs[vehicle].coords[tstep].steps:=tstep;


           //If the robot tries to move over the target position of another
           //robot (with him there), in some layer, this situation is detected
           //so, it's possible to change the priorities of the robots and avoid it;
           //Another solution is consider the target position as obstacle in
           //every layer starting at the step that reaches the goal until the
           //maximum NUM_LAYERS: the robot would plan an alternative path
           //however more inefficient
           //If trocas exceed the MAX_EXCHANGES it's considered that is not
           //possible optimize the paths
           if ((trocas < MAX_EXCHANGES) and (flagTargetOverlapInverse = false)) then begin
             v:=0;
             while v < vehicle do begin
                 if ((i=agvs[v].target_node) and (tstep>CaminhosAgvs[v].steps))
                 then begin
                       flagTargetOverlap:=true;
                       break;
                 end;
                 v:=v+1;
             end;
           end;

           count:=count+1;
    end;

    //maximum layers were exceeded and no path found
    if (((i<>agvs[vehicle].target_node) or
        (tstep<>0)) and (count = NUM_LAYERS))
    then begin
      errorMessage:=3;
    end;

    //update the path directions in each step, from the step 0 to the end
    d:=get_int_dirr(agvs[vehicle].Direction);
    CaminhosAgvs[vehicle].coords[0].direction:=d;

    UpdatePathDirections(CaminhosAgvs,vehicle, map);

end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure CloserNeighborhoodAsObstacle (var Map:TAStarMap; i:integer; tstep:integer);
var
    l1:integer;
    aux1:integer;
    ntl:integer;
begin
      l1:=length(Map.TEA_GRAPH[i-1][tstep].links);
      for aux1:=0 to l1-1 do
      begin
        ntl:=Map.TEA_GRAPH[i-1][tstep].links[aux1].node_to_link;
       if Map.GraphState[ntl-1][tstep] <> OBSTACLEWALL then Map.GraphState[ntl-1][tstep]:=OBSTACLEROBOT;
      end;

      //if(((i mod 2) = 0) and ((j mod 2) = 0)) then begin
     //   //it's a center point of a cell
     //   if Map.GridState[i+1][j][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j][tstep]:=OBSTACLEROBOT;
     //   if Map.GridState[i][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i][j+1][tstep]:=OBSTACLEROBOT;
     //   if Map.GridState[i-1][j][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j][tstep]:=OBSTACLEROBOT;
     //   if Map.GridState[i][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i][j-1][tstep]:=OBSTACLEROBOT;
     //end
     //else begin  //it's an edge
     //  if((i mod 2) = 0) then begin        //verify if it is an horizontal edge
     //       if Map.GridState[i-1][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-1][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j+1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i][j+1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j+1][tstep]:=OBSTACLEROBOT;
     //  end
     //  else begin                               //verify if it is a vertical edge
     //       if Map.GridState[i-1][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-1][j][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-1][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j+1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j+1][tstep]:=OBSTACLEROBOT;
     //  end;
     //end;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure LargeNeighborhoodAsObstacle (var Map:TAStarMap; i:integer; j:integer; tstep:integer);
var
    l1,l2:integer;
    aux1,aux2:integer;
    ntl,ntl2:integer;
begin
      l1:=length(Map.TEA_GRAPH[i-1][tstep].links);
      for aux1:=0 to l1-1 do
      begin
        ntl:=Map.TEA_GRAPH[i-1][tstep].links[aux1].node_to_link;
       if Map.GraphState[ntl-1][tstep] <> OBSTACLEWALL then Map.GraphState[ntl-1][tstep]:=OBSTACLEROBOT;
        l2:=length(Map.TEA_GRAPH[ntl-1][tstep].links);
         for aux2:=0 to l2-1 do
        begin
          ntl2:=Map.TEA_GRAPH[ntl-1][tstep].links[aux2].node_to_link;
          if Map.GraphState[ntl2-1][tstep] <> OBSTACLEWALL then Map.GraphState[ntl2-1][tstep]:=OBSTACLEROBOT;
        end;
      end;
     // if(((i mod 2) = 0) and ((j mod 2) = 0)) then begin
     //   //it's a center point of a cell
     //   if Map.GridState[i+1][j][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j][tstep]:=OBSTACLEROBOT;
     //   if Map.GridState[i][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i][j+1][tstep]:=OBSTACLEROBOT;
     //   if Map.GridState[i-1][j][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j][tstep]:=OBSTACLEROBOT;
     //   if Map.GridState[i][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i][j-1][tstep]:=OBSTACLEROBOT;
     //   if Map.GridState[i+1][j][tstep] <> OBSTACLEWALL then begin
     //       if Map.GridState[i+2][j][tstep] <> OBSTACLEWALL then Map.GridState[i+2][j][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+2][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i+2][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+2][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i+2][j+1][tstep]:=OBSTACLEROBOT;
     //   end;
     //   if Map.GridState[i-1][j][tstep] <> OBSTACLEWALL then begin
     //       if Map.GridState[i-2][j][tstep] <> OBSTACLEWALL then Map.GridState[i-2][j][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-2][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i-2][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-2][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i-2][j+1][tstep]:=OBSTACLEROBOT;
     //   end;
     //   if Map.GridState[i][j+1][tstep] <> OBSTACLEWALL then begin
     //       if Map.GridState[i][j+2][tstep] <> OBSTACLEWALL then Map.GridState[i][j+2][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-1][j+2][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j+2][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j+2][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j+2][tstep]:=OBSTACLEROBOT;
     //   end;
     //   if Map.GridState[i][j-1][tstep] <> OBSTACLEWALL then begin
     //       if Map.GridState[i][j-2][tstep] <> OBSTACLEWALL then Map.GridState[i][j-2][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-1][j-2][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j-2][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j-2][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j-2][tstep]:=OBSTACLEROBOT;
     //   end;
     //end
     //else begin  //it's an edge
     //  if((i mod 2) = 0) then begin        //verify if it is an horizontal edge
     //       if Map.GridState[i-1][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-1][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j+1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i][j+1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j+1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j][tstep] <> OBSTACLEWALL then begin
     //           if Map.GridState[i+2][j][tstep] <> OBSTACLEWALL then Map.GridState[i+2][j][tstep]:=OBSTACLEROBOT;
     //           if Map.GridState[i+2][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i+2][j-1][tstep]:=OBSTACLEROBOT;
     //           if Map.GridState[i+2][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i+2][j+1][tstep]:=OBSTACLEROBOT;
     //       end;
     //       if Map.GridState[i-1][j][tstep] <> OBSTACLEWALL then begin
     //           if Map.GridState[i-2][j][tstep] <> OBSTACLEWALL then Map.GridState[i-2][j][tstep]:=OBSTACLEROBOT;
     //           if Map.GridState[i-2][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i-2][j-1][tstep]:=OBSTACLEROBOT;
     //           if Map.GridState[i-2][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i-2][j+1][tstep]:=OBSTACLEROBOT;
     //       end;
     //       if Map.GridState[i][j-1][tstep] <> OBSTACLEWALL then begin
     //           if Map.GridState[i][j-2][tstep] <> OBSTACLEWALL then Map.GridState[i][j-2][tstep]:=OBSTACLEROBOT;
     //       end;
     //       if Map.GridState[i][j+1][tstep] <> OBSTACLEWALL then begin
     //           if Map.GridState[i][j+2][tstep] <> OBSTACLEWALL then Map.GridState[i][j+2][tstep]:=OBSTACLEROBOT;
     //       end;
     //  end
     //  else begin                               //verify if it is a vertical edge
     //       if Map.GridState[i-1][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-1][j][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i-1][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j+1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j-1][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j-1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i+1][j+1][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j+1][tstep]:=OBSTACLEROBOT;
     //       if Map.GridState[i][j+1][tstep] <> OBSTACLEWALL then begin
     //           if Map.GridState[i][j+2][tstep] <> OBSTACLEWALL then Map.GridState[i][j+2][tstep]:=OBSTACLEROBOT;
     //           if Map.GridState[i-1][j+2][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j+2][tstep]:=OBSTACLEROBOT;
     //           if Map.GridState[i+1][j+2][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j+2][tstep]:=OBSTACLEROBOT;
     //       end;
     //       if Map.GridState[i][j-1][tstep] <> OBSTACLEWALL then begin
     //           if Map.GridState[i][j-2][tstep] <> OBSTACLEWALL then Map.GridState[i][j-2][tstep]:=OBSTACLEROBOT;
     //           if Map.GridState[i-1][j-2][tstep] <> OBSTACLEWALL then Map.GridState[i-1][j-2][tstep]:=OBSTACLEROBOT;
     //           if Map.GridState[i+1][j-2][tstep] <> OBSTACLEWALL then Map.GridState[i+1][j-2][tstep]:=OBSTACLEROBOT;
     //       end;
     //       if Map.GridState[i-1][j][tstep] <> OBSTACLEWALL then begin
     //           if Map.GridState[i-2][j][tstep] <> OBSTACLEWALL then Map.GridState[i-2][j][tstep]:=OBSTACLEROBOT;
     //       end;
     //       if Map.GridState[i+1][j][tstep] <> OBSTACLEWALL then begin
     //           if Map.GridState[i+2][j][tstep] <> OBSTACLEWALL then Map.GridState[i+2][j][tstep]:=OBSTACLEROBOT;
     //       end;
     //  end;
     //end;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure UpdatePathDirections (var CaminhosAgvs:Caminhos; vehicle:integer; map:TAStarMap);
var
    diffx,diffy,diffx2,diffy2:double;
    id1,id2,id3:integer;
    count:integer;
begin
    count:=0;
    while count < CaminhosAgvs[vehicle].steps do begin
      id1:= CaminhosAgvs[vehicle].coords[count].node;
      id2:=CaminhosAgvs[vehicle].coords[count+1].node;
      id3:=CaminhosAgvs[vehicle].coords[count+2].node;

      diffx:=map.TEA_GRAPH[id2-1][0].pos_X - map.TEA_GRAPH[id1-1][0].pos_X;
      diffy:=map.TEA_GRAPH[id2-1][0].pos_Y - map.TEA_GRAPH[id1-1][0].pos_Y;
      diffx2:=map.TEA_GRAPH[id3-1][0].pos_X - map.TEA_GRAPH[id1-1][0].pos_X;
      diffy2:=map.TEA_GRAPH[id3-1][0].pos_Y - map.TEA_GRAPH[id1-1][0].pos_Y;

      if ((diffx = 0) and (diffy = 0)) then begin
          if ((diffx2 = 0) and (diffy2 = 1)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 0; end
          else if ((diffx2 = 0) and (diffy2 < 0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 4; end
          else if ((diffx2 >0) and (diffy2 = 0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 2; end
          else if ((diffx2 <0) and (diffy2 = 0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 6; end
          else if ((diffx2 >0) and (diffy2 >0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 1; end
          else if ((diffx2 >0) and (diffy2 <0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 3; end
          else if ((diffx2 <0) and (diffy2 >0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 7; end
          else if ((diffx2 <0) and (diffy2 <0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 5; end;
      end
      else if ((diffx = 0) and (diffy >0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 0; end
      else if ((diffx = 0) and (diffy <0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 4; end
      else if ((diffx > 0) and (diffy = 0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 2; end
      else if ((diffx <0) and (diffy = 0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 6; end
      else if ((diffx >0) and (diffy >0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 1; end
      else if ((diffx >0) and (diffy <0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 3; end
      else if ((diffx <0) and (diffy >0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 7; end
      else if ((diffx <0) and (diffy <0)) then begin CaminhosAgvs[vehicle].coords[count+1].direction := 5; end;
      count:=count+1;
    end;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure FreeCellsToVirgin (var Map:TAStarMap);
var
    tstep,i,j:integer;
begin

  tstep:=0;
  i:=0;

  //change the state of the non obstacles cells to VIRGIN in every layer
  //scan the nodes
  while i <= form1.graphsize do begin
          while tstep <= NUM_LAYERS do begin
              if ((Map.GraphState[i][tstep]<>OBSTACLEWALL) and (Map.GraphState[i][tstep] <> OBSTACLEROBOT))  then begin
                 Map.GraphState[i][tstep] := VIRGIN;
                 Map.TEA_GRAPH[i][tstep].G := 0;
                 Map.TEA_GRAPH[i][tstep].H := 0;
                 Map.TEA_GRAPH[i][tstep].Parent_node := 0;
                 Map.TEA_GRAPH[i][tstep].Parent_step := 0;
              end;
              tstep := tstep+1;
          end;
          tstep:=0;
      i:=i+1;
  end;

  tstep:=0;
  i:=0;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure CellsToVirgin(var Map:TAStarMap;startLayer:integer);
var
    tstep,i,j:integer;
begin

  tstep:=startLayer;
  i:=0;

  //change the state of the non obstacles cells to VIRGIN in every layer
  //scan the nodes
  while i <= form1.graphsize do begin
          while tstep <= NUM_LAYERS do begin
              if ((Map.GraphState[i][tstep]<>OBSTACLEWALL) and (Map.GraphState[i][tstep] <> OBSTACLEROBOT))  then begin
                 Map.GraphState[i][tstep] := VIRGIN;
                 Map.TEA_GRAPH[i][tstep].G := 0;
                 Map.TEA_GRAPH[i][tstep].H := 0;
                 Map.TEA_GRAPH[i][tstep].Parent_node := 0;
                 Map.TEA_GRAPH[i][tstep].Parent_step := 0;
              end;
              tstep := tstep+1;
          end;
          tstep:=startLayer;
      i:=i+1;
  end;

  tstep:=startLayer;
  i:=0;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure PathsToVirgin (var Map:TAStarMap);
var
    tstep,i,j:integer;
begin

  tstep:=0;
  i:=0;

  //change the state of the non obstacles cells to VIRGIN in every layer
  //scan the nodes
  while i <= form1.graphsize do begin
          while tstep <= NUM_LAYERS do begin
              if (Map.GraphState[i][tstep] = OBSTACLEROBOT) then begin
                 Map.GraphState[i][tstep] := VIRGIN;
                 Map.TEA_GRAPH[i][tstep].G := 0;
                 Map.TEA_GRAPH[i][tstep].H := 0;
                 Map.TEA_GRAPH[i][tstep].Parent_node := 0;
                 Map.TEA_GRAPH[i][tstep].Parent_step := 0;
              end;
              tstep := tstep+1;
          end;
          tstep:=0;
      i:=i+1;
  end;

  tstep:=0;
  i:=0;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure CleanHeapArray (var Map:TAStarMap);
var
  cleanArray: integer;
begin
  cleanArray := 0;

  Map.HeapArray.count:=0;
  Map.Profiler.Iter:=0;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
 function findindeposofnode(ntl:integer;Map:TAStarMap;l1:integer):integer;
 var
 aux1:integer;
 n_id:integer;
 r:integer;
 begin
 for aux1:=0 to l1-1 do begin
    n_id:=Map.TEA_GRAPH[aux1][0].id;
    if n_id=ntl then
    begin
      r:=n_id;
    end;
 end;
   findindeposofnode:=r;
 end;


//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
procedure InitialPositionAsObstacles (var vehicle: integer; Map: TAStarMap; agvs: r_node);
var
v,k,i,j:integer;
l1:integer;
aux1:integer;
ntl:integer;
ind:integer;
begin

    for v:=0 to (length(agvs)-1) do begin

        if(v<>vehicle) then begin

           //change the state of the cell to occupy in k={0,1}
           for k:=0 to 1 do begin
             i:=agvs[v].inicial_node;
             Map.GraphState[i-1][k] := OBSTACLEROBOT;
             l1:=length(Map.TEA_GRAPH[i-1][k].links);
             for aux1:=0 to l1-1 do begin
                 ntl:= Map.TEA_GRAPH[i-1][k].links[aux1].node_to_link;
                 ind:=findindeposofnode(ntl,Map,l1);
                 Map.GraphState[ind][k]:=OBSTACLEROBOT;
             end;
             //if(((i mod 2) = 0) and ((j mod 2) = 0)) then begin
             //     //it's a center point of a cell
             //     if Map.GridState[i+1][j][k] <> OBSTACLEWALL then Map.GridState[i+1][j][k]:=OBSTACLEROBOT;
             //     if Map.GridState[i][j+1][k] <> OBSTACLEWALL then Map.GridState[i][j+1][k]:=OBSTACLEROBOT;
             //     if Map.GridState[i-1][j][k] <> OBSTACLEWALL then Map.GridState[i-1][j][k]:=OBSTACLEROBOT;
             //     if Map.GridState[i][j-1][k] <> OBSTACLEWALL then Map.GridState[i][j-1][k]:=OBSTACLEROBOT;
             //  end
             //  else begin  //it's an edge
             //    if((i mod 2) = 0) then begin        //verify if it is an horizontal edge
             //         if Map.GridState[i-1][j-1][k] <> OBSTACLEWALL then Map.GridState[i-1][j-1][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i][j-1][k] <> OBSTACLEWALL then Map.GridState[i][j-1][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i+1][j-1][k] <> OBSTACLEWALL then Map.GridState[i+1][j-1][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i-1][j+1][k] <> OBSTACLEWALL then Map.GridState[i-1][j+1][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i][j+1][k] <> OBSTACLEWALL then Map.GridState[i][j+1][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i+1][j+1][k] <> OBSTACLEWALL then Map.GridState[i+1][j+1][k]:=OBSTACLEROBOT;
             //    end
             //    else begin                               //verify if it is a vertical edge
             //         if Map.GridState[i-1][j-1][k] <> OBSTACLEWALL then Map.GridState[i-1][j-1][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i-1][j][k] <> OBSTACLEWALL then Map.GridState[i-1][j][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i-1][j+1][k] <> OBSTACLEWALL then Map.GridState[i-1][j+1][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i+1][j-1][k] <> OBSTACLEWALL then Map.GridState[i+1][j-1][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i+1][j][k] <> OBSTACLEWALL then Map.GridState[i+1][j][k]:=OBSTACLEROBOT;
             //         if Map.GridState[i+1][j+1][k] <> OBSTACLEWALL then Map.GridState[i+1][j+1][k]:=OBSTACLEROBOT;
             //    end;

             //podera ser melhorado considerando posições intermédias do agv ou seja fora dos nodes


           end;
        end
        else
        begin
             //the cell continues free to the own vehicle
        end;

    end;

end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
 //procedure addtotable(table:link_table;n1:integer;n2:integer;l_id:integer;n_id:integer;dist:double);
 //var
 //ele:link_telement;
 //l1:integer;
 //l2:integer;
 //aux1:integer;
 //begin
 //   ele:=table.table[n1-1][n2-1];
 //   l1:=length(ele.id_l);
 //   setlength(ele.id_l,l1+1);
 //   ele.id_l[l1]:=l_id;
 //   l1:=length(ele.id_nodes);
 //   setlength(ele.id_nodes,l1+1);
 //   ele.id_nodes[l1]:=n_id;
 //   aux1:=ele.distance;
 //   ele.distance:=abs(aux1)+dist;
 //   table.table[n1-1][n2-1]:=ele;
 //end;
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------



//function Buildlinktable(var graph: a_node):link_table;
//var
//  l1:integer;
//  l2:integer;
//  l3:integer;
//  aux1:integer;
//  aux2:integer;
//  visited:array of integer;
//  f_visited:integer;
//  n_id:integer;
//  ntl:integer;
//  l_id:integer;
//  l_table:link_table;
//begin
//   l1:=length(graph);
//   setlength(l_table.table,l1,l1);
//   for aux1:=0 to l1-1 do
//   begin
//     l2:=length(graph[aux1].links);
//     n_id:=graph[aux1].id;
//     for aux2:=0 to l2-1 do
//     begin
//      ntl:=graph[aux1].links[aux2].node_to_link;
//      l_id:=graph[aux1].links[aux2].id_l;
//      //addtotable(table:link_table;n1:integer;n2:integer;l_id:integer;n_id:integer);
//     end;
//   end;
//end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

function CalcH(var Map: TAStarMap; Pi, Pf: integer): double; inline;
  var
    l2:integer;
    l3:integer;
    aux1:integer;
    visited:array of integer;
    f_visited:integer;
begin
 // array stores the H cost for each [x,y] that corresponds to the absolute differences
 // between the actual node and the target node
 // Result := CalcHCache[Abs(Pi.x - Pf.x), Abs(Pi.y - Pf.y)];

 // Result:= round(sqrt(sqr(Pi.x-Pf.x) + sqr(Pi.y-Pf.y)) * Map.EucliDistK * FixedPointConst);

 //Result:= round(sqrt(sqr(Pi.x-Pf.x) + sqr(Pi.y-Pf.y)));

 //Result:= sqrt(sqr(Pi.x-Pf.x) + sqr(Pi.y-Pf.y));
  //l2:=length(Map.TEA_GRAPH[Pi][0].links);
  //l3:=length(visited);
  //setlength(visited,l3+1);
  //visited[l3]:=Pi;
  //for aux1:=0 to l2-1 do
  //begin
  //  ntl:=Map.TEA_GRAPH[Pi][0].links[aux1].node_to_link;
  //
  //end;
  //

  Result := sqrt(sqr((Map.TEA_GRAPH[Pi-1][0].pos_X-Map.TEA_GRAPH[Pf-1][0].pos_X)/2) + sqr((Map.TEA_GRAPH[Pi-1][0].pos_Y-Map.TEA_GRAPH[Pf-1][0].pos_Y)/2));
  //divides the distance x and y by two to ensure coherence with costs of displacement

end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

function CalcF(var Map: TAStarMap; idx: integer): double; inline;
begin
  with Map.HeapArray.data[idx]^ do begin
    result := G + H;
  end;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure InsertInOpenList( var Map: TAStarMap; id: integer; step:integer);
var idx: integer;
  AStarHeapArraySize:integer;
begin
  AStarHeapArraySize := form1.graphsize*NUM_LAYERS;
  // verify if the size of the heap is not exceeded
  if Map.HeapArray.count >= AStarHeapArraySize then exit;
  inc(Map.Profiler.AddToOpenList_count);

  // update the grid state
  Map.GraphState[id-1][step]:= OPENED;

  // insert at the bottom of the heap
  idx := Map.HeapArray.count;
  Map.HeapArray.data[idx] := @Map.TEA_GRAPH[id-1][step];
  Map.TEA_GRAPH[id-1][step].HeapIdx := idx;
  Inc(Map.HeapArray.count);

  // update by promotion up to the right place
  UpdateHeapPositionByPromotion(Map, idx);
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure UpdateHeapPositionByPromotion(var Map: TAStarMap; idx: integer); inline;
var parent_idx: integer;
    node_cost: double;
begin
  // if we are on the first node, there is no way to promote
  if idx = 0 then exit;

  // calc node cost
  node_cost := CalcF(Map, idx);

  // repeat until we can promote no longer
  while true do begin

    // if we are on the first node, there is no way to promote
    if idx = 0 then exit;

    //parent_idx := (idx - 1) div 2;  // WHY DIV 2 ???
    parent_idx := idx-1;

    // if the parent is better than we are, there will be no promotion
    if CalcF(Map, parent_idx) < node_cost then exit;

    // if not, just promote it
    SwapHeapElements(Map, idx, parent_idx);
    idx := parent_idx;
  end;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure SwapHeapElements(var Map: TAStarMap; idx1, idx2: integer); inline;
var ptr1, ptr2: PAStarCell;
begin
  ptr1 := Map.HeapArray.data[idx1];   //points the adress of data of idx1 position
  ptr2 := Map.HeapArray.data[idx2];
  ptr1^.HeapIdx := idx2;              //stores the index where the ptr1 will be match
  ptr2^.HeapIdx := idx1;
  Map.HeapArray.data[idx1] := ptr2;   //swap the information
  Map.HeapArray.data[idx2] := ptr1;
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure RemoveFromOpenList(var Map: TAStarMap; out Pnt: integer); inline;
begin

  inc(Map.Profiler.RemoveFromOpenList_count);

  with Map.HeapArray do begin
    // return the first node
    Pnt := data[0]^.id;
    // move the last node into the first position
    data[count - 1]^.HeapIdx := 0;
    data[0] := data[count - 1];
    // update the array size
    Dec(count);
  end;
  // re-sort that "first" node (the node moved from the last position to the first position)
  UpdateHeapPositionByDemotion(Map,0);
end;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

procedure UpdateHeapPositionByDemotion(var Map: TAStarMap; idx: integer); inline;
var c1, c2, cost: double;
    idx_child, new_idx: integer;
begin

  cost := CalcF(Map, idx);

  while true do begin

    //idx_child := idx * 2 + 1;
    idx_child := idx+1;

    // if the node has no childs, there is no way to demote
    if idx_child >= Map.HeapArray.count then exit;

    // calc our cost and the first node cost
    c1 := CalcF(Map, idx_child);
    // if there is only one child, just compare with this one
    if idx_child + 1 >= Map.HeapArray.count then begin
      // if we are better than this child, then no demotion
      if cost < c1 then exit;
      // if not, then do the demotion
      SwapHeapElements(Map, idx, idx_child);
      exit;
    end;

    // calc the second node cost
    c2 := CalcF(Map, idx_child + 1);

    // select the best node to demote to
    new_idx := idx;
    if c2 < cost then begin
      if c1 < c2 then begin
        new_idx := idx_child;
      end else begin
        new_idx := idx_child + 1;
      end;
    end else if c1 < cost then begin
      new_idx := idx_child;
    end;

    // if there is no better child, just return
    if new_idx = idx then exit;

    // if we want to demote, then swap the elements
    SwapHeapElements(Map, idx, new_idx);
    idx := new_idx;
  end;
end;

end.




