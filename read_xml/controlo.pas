unit controlo;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, lNetComponents, Forms, Controls, Graphics,
  Dialogs, StdCtrls, ExtCtrls, lNet, TEAstar, laz2_DOM, laz2_XMLRead, math, Utils3DS,Utils;

const
  THRESHOLD_DIST = 0.03;//0.025;
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

  TFControlo = class(TForm)
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
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
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
procedure MovementDecision(var CaminhosAgvs:Caminhos;var agvs:r_node;thetaCam:double;i:integer;Form:TFControlo);
procedure UnpackUDPmessage(var xCam:array of double;var yCam:array of double; var thetaCam:array of double;data:string);
procedure UpdateInitialPoints(var xCam:array of double;var yCam:array of double; var thetaCam:array of double);
procedure PositionAndOrientationControl(i:integer;thetaCam:array of double;dist:double;angle:double);
procedure NextDirectionToThetaDest(var CaminhosAgvs:Caminhos;i:integer);
procedure DefineRotationCenterPoint(var CaminhosAgvs:Caminhos;i:integer);
procedure UpdateSubmissions(var agvs:r_node;i:integer);

var
  FControlo: TFControlo;
  MessageInitialPositions: string;
  MessageVelocities: string;
  Map: TAStarMap;
  agvs: r_node;
  CaminhosAgvs: Caminhos;
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

implementation
 uses
   unit1;
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
                                 floatToStr(agvs[v].ipos_X) + 'Y' +
                                 floatToStr(agvs[v].ipos_Y) + 'D' +
                                 floatToStr(agvs[v].iDirection);
      xDest[v]:=agvs[v].ipos_X;
      yDest[v]:=agvs[v].ipos_Y;
      v:=v+1;
    end;
    MessageInitialPositions := MessageInitialPositions + 'F';
end;

procedure ChangeRobotPriorities(var Map:TEAstar.TAStarMap;var agvs:r_node);
var
    aux_agv: TEAstar.Robot_Pos_info;
    aux_xDest,aux_yDest: double;

begin
        noPath:=false;
        trocas:=trocas+1;

        ListPriorities[agvs[robotNoPlan].InitialIdPriority]:=robotNoPlan-1;
        ListPriorities[agvs[robotNoPlan-1].InitialIdPriority]:=robotNoPlan;

        aux_agv := agvs[robotNoPlan];
        agvs[robotNoPlan] := agvs[robotNoPlan-1];
        agvs[robotNoPlan-1] := aux_agv;

        aux_xDest := xDest[robotNoPlan];
        xDest[robotNoPlan] := xDest[robotNoPlan-1];
        xDest[robotNoPlan-1] := aux_xDest;

        aux_yDest := yDest[robotNoPlan];
        yDest[robotNoPlan] := yDest[robotNoPlan-1];
        yDest[robotNoPlan-1] := aux_yDest;

        robotNoPlan := A_starTimeGo(Map,CaminhosAgvs,agvs,MAX_ITERATIONS);

end;

procedure InverseValidationOfPriorities(var Map:TAStarMap;var agvs:r_node;var CaminhosAgvs:Caminhos);
var
    stepPath: integer;
    v,robo: integer;
    i,j,k,count,steps: integer;
    aux_agv: TEAstar.Robot_Pos_info;
    aux_xDest,aux_yDest: double;
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

                ListPriorities[agvs[robotNoPlan].InitialIdPriority]:=robotNoPlan-1;
                ListPriorities[agvs[robotNoPlan-1].InitialIdPriority]:=robotNoPlan;

                aux_agv := agvs[robotNoPlan];
                agvs[robotNoPlan] := agvs[robotNoPlan-1];
                agvs[robotNoPlan-1] := aux_agv;

                aux_xDest := xDest[robotNoPlan];
                xDest[robotNoPlan] := xDest[robotNoPlan-1];
                xDest[robotNoPlan-1] := aux_xDest;

                aux_yDest := yDest[robotNoPlan];
                yDest[robotNoPlan] := yDest[robotNoPlan-1];
                yDest[robotNoPlan-1] := aux_yDest;

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

procedure MovementDecision(var CaminhosAgvs:Caminhos;var agvs:r_node;thetaCam:double;i:integer;Form:TFControlo);
begin
        //Encontra-se na célula objetivo
        if (CaminhosAgvs[i].coords[0].node = agvs[i].target_node)
        then begin
            Form.Edit6.Text:='TARGET';
            linearVelocities[i]:=0;
            angularVelocities[i]:=0;
            followLine[i]:=false;
            followCircle[i]:=false;
            rotate[i]:=false;
        end

        else if (CaminhosAgvs[i].coords[0].node = agvs[i].SubMissions[agvs[i].NumberSubMissions-1])
        then begin
            linearVelocities[i]:=0;
            angularVelocities[i]:=0;
            followLine[i]:=false;
            followCircle[i]:=false;
            rotate[i]:=false;
        end

        //Mudança de célula em 1 Step
        else if (CaminhosAgvs[i].coords[1].node <> CaminhosAgvs[i].coords[0].node)
        then begin
            xDest[i]:=getXcoord(CaminhosAgvs[i].coords[1].node);
            yDest[i]:=getYcoord(CaminhosAgvs[i].coords[1].node);
            if (abs(thetaDest[i]-thetaCam) < THRESHOLD_ANGLE) then begin
                linearVelocities[i]:=VNOM;
                angularVelocities[i]:=0;
                followLine[i]:=true;
            end
            else if ((thetaDest[i]-thetaCam) < (-pi*2 + THRESHOLD_ANGLE)) then begin
                linearVelocities[i]:=VNOM;
                angularVelocities[i]:=0;
                followLine[i]:=true;
            end
            else if ((thetaDest[i]-thetaCam < -pi + THRESHOLD_ANGLE) and
                     (thetaDest[i]-thetaCam > -pi - THRESHOLD_ANGLE))
            then begin
                linearVelocities[i]:=-VNOM;
                angularVelocities[i]:=0;
                followLine[i]:=true;
                UpdateThetaDestPi(i,thetaCam);
            end
            else if ((thetaDest[i]-thetaCam < pi + THRESHOLD_ANGLE) and
                     (thetaDest[i]-thetaCam > pi - THRESHOLD_ANGLE))
            then begin
                linearVelocities[i]:=-VNOM;
                angularVelocities[i]:=0;
                followLine[i]:=true;
                UpdateThetaDestPi(i,thetaCam);
            end
            else if ((thetaDest[i]-thetaCam < -pi/4 + THRESHOLD_ANGLE) and
                     (thetaDest[i]-thetaCam > -pi/4 - THRESHOLD_ANGLE))
            then begin
                linearVelocities[i]:=VNOM;
                angularVelocities[i]:=0;
                directionToFollow[i]:=1;
                followLine[i]:=true;
            end
            else if ((thetaDest[i]-thetaCam < -pi/4 + THRESHOLD_ANGLE) and
                     (thetaDest[i]-thetaCam > -pi/4 - THRESHOLD_ANGLE))
            then begin
                linearVelocities[i]:=0;
                angularVelocities[i]:=WNOM;
                directionToFollow[i]:=1;
                rotate[i]:=true;
            end
            else if ((thetaDest[i]-thetaCam < pi/4 + THRESHOLD_ANGLE) and
                     (thetaDest[i]-thetaCam > pi/4 - THRESHOLD_ANGLE))
            then begin
                linearVelocities[i]:=0;
                angularVelocities[i]:=-WNOM;
                directionToFollow[i]:=1;
                rotate[i]:=true;
            end
            else if ((thetaDest[i]-thetaCam < -pi*0.75 + THRESHOLD_ANGLE) and
                     (thetaDest[i]-thetaCam > -pi*0.75 - THRESHOLD_ANGLE))
            then begin
                linearVelocities[i]:=0;
                angularVelocities[i]:=-WNOM;
                directionToFollow[i]:=2; //anda de marcha atrás
                UpdateThetaDestPi(i,thetaCam);
                rotate[i]:=true;
            end
            else if ((thetaDest[i]-thetaCam < pi*0.75 + THRESHOLD_ANGLE) and
                     (thetaDest[i]-thetaCam > pi*0.75 - THRESHOLD_ANGLE))
            then begin
                linearVelocities[i]:=0;
                angularVelocities[i]:=WNOM;
                directionToFollow[i]:=2; //anda de marcha atrás
                UpdateThetaDestPi(i,thetaCam);
                rotate[i]:=true;
             end
             else if ((thetaDest[i]-thetaCam < -pi*1.25 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > -pi*1.25 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=0;
                 angularVelocities[i]:=WNOM;
                 directionToFollow[i]:=2; //anda de marcha atrás
                 UpdateThetaDestPi(i,thetaCam);
                 rotate[i]:=true;
             end
             else if ((thetaDest[i]-thetaCam < pi*1.25 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > pi*1.25 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=0;
                 angularVelocities[i]:=-WNOM;
                 directionToFollow[i]:=2; //anda de marcha atrás
                 UpdateThetaDestPi(i,thetaCam);
                 rotate[i]:=true;
             end
             else if ((thetaDest[i]-thetaCam < -pi*1.75 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > -pi*1.75 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=0;
                 angularVelocities[i]:=-WNOM;
                 directionToFollow[i]:=1;
                 rotate[i]:=true;
             end;
        end

        //Situação de Espera
        else if ((CaminhosAgvs[i].coords[1].node = CaminhosAgvs[i].coords[0].node) and
                 (directionDest[i] = CaminhosAgvs[i].coords[0].direction))
        then begin
             xDest[i]:=getXcoord(CaminhosAgvs[i].coords[1].node);
             yDest[i]:=getYcoord(CaminhosAgvs[i].coords[1].node);
             linearVelocities[i]:=0;
             angularVelocities[i]:=0;
             followLine[i]:=true;
        end

        //Para considerar o robô parado e arrancar posteriormente de marcha atrás
        //Ao invés de entrar na condição da mudança de célula em 2 steps
        else if ((CaminhosAgvs[i].coords[1].node = CaminhosAgvs[i].coords[0].node) and
                 ((directionDest[i] - CaminhosAgvs[i].coords[0].direction = 4) or
                  (CaminhosAgvs[i].coords[0].direction - directionDest[i] = 4)))
        then begin
             linearVelocities[i]:=0;
             angularVelocities[i]:=0;
        end

        //Mudança de célula em 2 Steps
        else if (CaminhosAgvs[i].coords[1].node = CaminhosAgvs[i].coords[0].node)
        then begin
             xDest[i]:=getXcoord(CaminhosAgvs[i].coords[2].node);
             yDest[i]:=getYcoord(CaminhosAgvs[i].coords[2].node);

             if ((thetaDest[i]-thetaCam < -pi/4 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > -pi/4 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=VNOM;
                 angularVelocities[i]:=linearVelocities[i]/RAIO_CURVATURA;
                 wf[i]:=angularVelocities[i];
                 followCircle[i]:=true;
                 directionToFollow[i]:=1;
                 UpdateThetaDest(i,thetaCam);
                 Form.Edit7.Text:='1';
             end
             else if ((thetaDest[i]-thetaCam < pi/4 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > pi/4 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=VNOM;
                 angularVelocities[i]:=-linearVelocities[i]/RAIO_CURVATURA;
                 wf[i]:=angularVelocities[i];
                 followCircle[i]:=true;
                 directionToFollow[i]:=1;
                 UpdateThetaDest(i,thetaCam);
                 Form.Edit7.Text:='8753';
             end
             else if ((thetaDest[i]-thetaCam < -pi*0.75 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > -pi*0.75 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=-VNOM;
                 angularVelocities[i]:=+linearVelocities[i]/RAIO_CURVATURA;
                 wf[i]:=angularVelocities[i];
                 followCircle[i]:=true;
                 directionToFollow[i]:=2;
                 //UpdateThetaDest(i,thetaCam);
                 //UpdateThetaDestPi(i,thetaCam);
                 UpdateThetaDestToMoveBack(i,thetaCam);
                 Form.Edit7.Text:='3';
             end
             else if ((thetaDest[i]-thetaCam < pi*0.75 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > pi*0.75 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=-VNOM;
                 angularVelocities[i]:=-linearVelocities[i]/RAIO_CURVATURA;
                 wf[i]:=angularVelocities[i];
                 followCircle[i]:=true;
                 directionToFollow[i]:=2;
                 //UpdateThetaDest(i,thetaCam);
                 //UpdateThetaDestPi(i,thetaCam);
                 UpdateThetaDestToMoveBack(i,thetaCam);
                 Form.Edit7.Text:='4';
             end
             else if ((thetaDest[i]-thetaCam < -pi*1.25 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > -pi*1.25 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=-VNOM;
                 angularVelocities[i]:=-linearVelocities[i]/RAIO_CURVATURA;
                 wf[i]:=angularVelocities[i];
                 followCircle[i]:=true;
                 directionToFollow[i]:=2;
                 //UpdateThetaDestPi(i,thetaCam);
                 //UpdateThetaDest(i,thetaCam);
                 //UpdateThetaDestPi(i,thetaCam);
                 UpdateThetaDestToMoveBack(i,thetaCam);
                 Form.Edit7.Text:='5';
             end
             else if ((thetaDest[i]-thetaCam < pi*1.25 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > pi*1.25 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=-VNOM;
                 angularVelocities[i]:=+linearVelocities[i]/RAIO_CURVATURA;
                 wf[i]:=angularVelocities[i];
                 followCircle[i]:=true;
                 directionToFollow[i]:=2;
                 //UpdateThetaDestPi(i,thetaCam);
                 //UpdateThetaDest(i,thetaCam);
                 //UpdateThetaDestPi(i,thetaCam);
                 UpdateThetaDestToMoveBack(i,thetaCam);
                 Form.Edit7.Text:='6';
             end
             else if ((thetaDest[i]-thetaCam < pi*1.75 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > pi*1.75 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=VNOM;
                 angularVelocities[i]:=linearVelocities[i]/RAIO_CURVATURA;
                 wf[i]:=angularVelocities[i];
                 followCircle[i]:=true;
                 directionToFollow[i]:=1;
                 UpdateThetaDestPi(i,thetaCam);
                 UpdateThetaDestAfterPiRotation(i,thetaCam);
                 Form.Edit7.Text:='7';
             end
             else if ((thetaDest[i]-thetaCam < -pi*1.75 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > -pi*1.75 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=VNOM;
                 angularVelocities[i]:=-linearVelocities[i]/RAIO_CURVATURA;
                 wf[i]:=angularVelocities[i];
                 followCircle[i]:=true;
                 directionToFollow[i]:=1;
                 UpdateThetaDestPi(i,thetaCam);
                 //UpdateThetaDest(i,thetaCam)
                 UpdateThetaDestAfterPiRotation(i,thetaCam);
                 Form.Edit7.Text:='8754';
             end
             else if ((thetaDest[i]-thetaCam < -pi/2 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > -pi/2 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=0;
                 angularVelocities[i]:=WNOM;
                 rotate[i]:=true;
                 directionToFollow[i]:=1;
                 Form.Edit7.Text:='9';
             end
             else if ((thetaDest[i]-thetaCam < pi/2 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > pi/2 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=0;
                 angularVelocities[i]:=-WNOM;
                 rotate[i]:=true;
                 directionToFollow[i]:=1;
                 Form.Edit7.Text:='10';
             end
             else if ((thetaDest[i]-thetaCam < -pi*1.5 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > -pi*1.5 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=0;
                 angularVelocities[i]:=-WNOM;
                 rotate[i]:=true;
                 directionToFollow[i]:=1;
                 Form.Edit7.Text:='11';
             end
             else if ((thetaDest[i]-thetaCam < pi*1.5 + THRESHOLD_ANGLE) and
                      (thetaDest[i]-thetaCam > pi*1.5 - THRESHOLD_ANGLE))
             then begin
                 linearVelocities[i]:=0;
                 angularVelocities[i]:=WNOM;
                 rotate[i]:=true;
                 directionToFollow[i]:=1;
                 Form.Edit7.Text:='12';
             end;


             DefineRotationCenterPoint(CaminhosAgvs,i);
             if (((wf[i] > 0) and (directionToFollow[i] = 1)) or
                 ((wf[i] > 0) and (directionToFollow[i] = 2)))
             then begin
                s[i].x:=(getXcoord(CaminhosAgvs[i].coords[0].node)-1)*CELLSCALE - rotationCenter[i].x;
                s[i].y:=(getYcoord(CaminhosAgvs[i].coords[0].node)-1)*CELLSCALE - rotationCenter[i].y;
             end
             else if (((wf[i] < 0) and (directionToFollow[i] = 1)) or
                      ((wf[i] < 0) and (directionToFollow[i] = 2)))
             then begin
                s[i].x:=rotationCenter[i].x - (getXcoord(CaminhosAgvs[i].coords[0].node)-1)*CELLSCALE;
                s[i].y:=rotationCenter[i].y - (getycoord(CaminhosAgvs[i].coords[0].node)-1)*CELLSCALE;
             end;
        end;
end;

procedure UnpackUDPmessage(var xCam:array of double;var yCam:array of double; var thetaCam:array of double;data:string);
var
    i,j:integer;
    xCamStr,yCamStr,thetaCamStr:string;
begin
    i:=0;
    j:=1;
    if (data<>'') then begin
         while (i<NUMBER_ROBOTS) do begin

            j:=j+2;
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
            while ((data[j]<>'R') and (data[j]<>'F')) do begin
               thetaCamStr := thetaCamStr + data[j];
               j:=j+1;
            end;

            xCam[ListPriorities[i]] := StrToFloat(xCamStr);
            yCam[ListPriorities[i]] := StrToFloat(yCamStr);
            thetaCam[ListPriorities[i]] := StrToFloat(thetaCamStr);

            xCamStr:='';
            yCamStr:='';
            thetaCamStr:='';

            i:=i+1;
         end;
    end;
end;

procedure UpdateInitialPoints(var xCam:array of double;var yCam:array of double; var thetaCam:array of double);
var
    i:integer;
begin
    i:=0;
    while i<NUMBER_ROBOTS do begin
        form1.robots[i].pos_X := round((xCam[i]+CELLSCALE)/CELLSCALE);
         form1.robots[i].pos_Y:= round((yCam[i]+CELLSCALE)/CELLSCALE);
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
        i:=i+1;
    end;
end;

procedure PositionAndOrientationControl(i:integer;thetaCam:array of double;dist:double;angle:double);
begin
    if (followLine[i]=true) then begin
        angularVelocities[i] := -GANHO_DIST*dist-GANHO_THETA*angle;
    end

    else if (followCircle[i]=true) then begin
        if ((wf[i] > 0) and (directionToFollow[i] = 1)) then begin
             angularVelocities[i] := +GANHO_DIST_CIRCLE*dist+GANHO_THETA_CIRCLE*angle + wf[i];
        end
        else if ((wf[i] < 0) and (directionToFollow[i] = 1)) then begin
             angularVelocities[i] := -GANHO_DIST_CIRCLE*dist +GANHO_THETA_CIRCLE*angle + wf[i];
        end
        else if ((wf[i] < 0) and (directionToFollow[i] = 2)) then begin
             angularVelocities[i] := -GANHO_DIST_CIRCLE_BACK*dist +GANHO_THETA_CIRCLE_BACK*angle + wf[i];
        end
        else if ((wf[i] > 0) and (directionToFollow[i] = 2)) then begin
             angularVelocities[i] := +GANHO_DIST_CIRCLE_BACK*dist +GANHO_THETA_CIRCLE_BACK*angle + wf[i];
        end;
    end

    else if (rotate[i]=true) then begin
        if (abs(thetaCam[i]-thetaDest[i]) <= THRESHOLD_ANGLE) then begin
            rotate[i]:=false;
            followLine[i]:=true;
            if directionToFollow[i] = 1 then begin
               linearVelocities[i]:=VNOM;
            end
            else if directionToFollow[i] = 2 then begin
               linearVelocities[i]:=-VNOM;
            end;
            angularVelocities[i]:=0;
        end;
    end;
end;

procedure NextDirectionToThetaDest(var CaminhosAgvs:Caminhos;i:integer);
begin
      //os ângulos correspondentes às direções foram feitos com
      //base no refernecial do SimTwo
      case CaminhosAgvs[i].coords[1].direction of
      0: begin
              thetaDest[i]:=pi/2;
         end;
      1: begin
              thetaDest[i]:=pi/4;
         end;
      2: begin
              thetaDest[i]:=0;
         end;
      3: begin
              thetaDest[i]:=-pi/4;
         end;
      4: begin
              thetaDest[i]:=-pi/2;
         end;
      5: begin
              thetaDest[i]:=-pi*0.75;
         end;
      6: begin
              thetaDest[i]:=-pi;
         end;
      7: begin
              thetaDest[i]:=pi*0.75;
         end;
      end;
end;

procedure DefineRotationCenterPoint(var CaminhosAgvs:Caminhos;i:integer);
begin
    rotationCenter[i].x := getXcoord(CaminhosAgvs[i].coords[0].node)*CELLSCALE;
    rotationCenter[i].y := getYcoord(CaminhosAgvs[i].coords[0].node)*CELLSCALE;
end;


procedure UpdateSubmissions(var agvs:R_NODE;i:integer);
begin
    if ((agvs[i].inicial_node <> agvs[i].SubMissions[agvs[i].NumberSubMissions-1]))
    then begin

        if ((agvs[i].inicial_node = agvs[i].SubMissions[agvs[i].ActualSubMission-1]) and
            (agvs[i].ActualSubMission <> agvs[i].NumberSubMissions))
        then begin
            agvs[i].ActualSubMission := agvs[i].ActualSubMission + 1;
            agvs[i].CounterSubMissions := agvs[i].CounterSubMissions + 1;
        end;

    end;
end;

{ TFControlo }

procedure TFControlo.udpComReceive(aSocket: TLSocket);
var
    data : string;
    i : integer;
    xCam,yCam,thetaCam: array[0..NUMBER_ROBOTS-1] of double;
begin
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

          UnpackUDPmessage(xCam,yCam,thetaCam,data);

          UpdateInitialPoints(xCam,yCam,thetaCam);

          //i:=0;
          //thetaDest[i]:=-pi;
          //xDest[i]:=17;
          //yDest[i]:=2;
          //if (((abs(xCam[i] - (xDest[i]-1)*CELLSCALE)) <= THRESHOLD_DIST) and ((abs(yCam[i] - (yDest[i]-1)*CELLSCALE)) <= THRESHOLD_DIST)) then begin
          //    linearVelocities[i]:=0;
          //    angularVelocities[i]:=0;
          //end
          //else begin
          //    followLine[i] := false;
          //    followCircle[i] := true;
          //    rotationCenter[i].x:=(17-1)*CELLSCALE;
          //    rotationCenter[i].y:=(3-1)*CELLSCALE;
          //    s[i].x:=rotationCenter[i].x - (16-1)*CELLSCALE;
          //    s[i].y:=rotationCenter[i].y - (3-1)*CELLSCALE;
          //    dist := DistToReference(i,xCam[i],yCam[i]);
          //    angle := AngleToReference(i,xCam[i],yCam[i],thetaCam[i]);
          //    linearVelocities[i] := -VNOM;
          //    angularVelocities[i] := -GANHO_DIST_CIRCLE_BACK*dist +GANHO_THETA_CIRCLE_BACK*angle + linearVelocities[i]/RAIO_CURVATURA;
          //    i:=0;
          //end;

          i:=0;
          while i<NUMBER_ROBOTS do begin
              if ((((abs(xCam[i] - (xDest[i])*CELLSCALE)) > THRESHOLD_DIST) or ((abs(yCam[i] - (yDest[i])*CELLSCALE)) > THRESHOLD_DIST)))
              then begin
                  Edit4.Text:='MOVE';
                  //detetar falhas
                  //dist := DistToReference(i,xCam[i],yCam[i]);
                  //angle := AngleToReference(i,xCam[i],yCam[i],thetaCam[i]);
                  //Edit5.Text:=floattostr(xCam[i]);
                  //Edit6.Text:=floattostr(yCam[i]);
                  //Edit7.Text:=floattostr(xDest[i]);
                  //Edit8.Text:=floattostr(yDest[i]);
                  //Edit9.Text:=floattostr(dist);
                  //PositionAndOrientationControl(i,thetaCam,dist,angle);

              end
              else if (((abs(xCam[i] - (xDest[i])*CELLSCALE)) <= THRESHOLD_DIST) and ((abs(yCam[i] - (yDest[i])*CELLSCALE)) <= THRESHOLD_DIST)) then begin

                  Edit4.Text:='TEA';
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

                  directionDest[i] := CaminhosAgvs[i].coords[1].direction;

                  NextDirectionToThetaDest(CaminhosAgvs,i);

                  MovementDecision(CaminhosAgvs,form1.robots,thetaCam[i],i,FControlo);

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
end;

procedure TFControlo.TimerSendTimer(Sender: TObject);
var
    i: integer;
    l1:integer;
    aux1:integer;
begin
    if flagMessageInitialPositions = true then begin
       udpCom.SendMessage(MessageInitialPositions, '127.0.0.1:9808');
    end;

    if flagVelocities = true then begin
        i:=0;
        while i<NUMBER_ROBOTS do begin
           MessageVelocities := MessageVelocities + 'P' + IntToStr( form1.robots[i].InitialIdPriority);
           MessageVelocities := MessageVelocities + 'S' + IntToStr( CaminhosAgvs[i].steps+1);
           l1:=length((CaminhosAgvs[i].coords));
           for aux1:=0 to l1-1 do begin
           if ((getXcoord(CaminhosAgvs[i].coords[aux1].node)<3) and (getYcoord(CaminhosAgvs[i].coords[aux1].node)<3)) then
           begin
           MessageVelocities := MessageVelocities + 'I' + IntToStr(aux1+1)
                                                  + 'X' + FloatToStr(round2(getXcoord(CaminhosAgvs[i].coords[aux1].node),3))
                                                  + 'Y' + FloatToStr(round2(getYcoord(CaminhosAgvs[i].coords[aux1].node),3))
                                                  + 'D' + IntToStr(CaminhosAgvs[i].coords[aux1].direction);
           end;
           end;
           i:=i+1;
        end;
        MessageVelocities := MessageVelocities + 'F';
        udpCom.SendMessage(MessageVelocities, '127.0.0.1:9808');
        Edit8.Text:=MessageVelocities;
        MessageVelocities:='';
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

    InitialPointsForAllRobots(form1.robots);


    i:=0;
    while i<NUMBER_ROBOTS do begin
      linearVelocities[i]:=0;
      angularVelocities[i]:=0;
      i:=i+1;
    end;

end;

procedure TFControlo.SendButtonClick(Sender: TObject);
var
    i:integer;
begin
    flagVelocities:=true;
end;

procedure TFControlo.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
    udpCom.Disconnect();
end;

end.
