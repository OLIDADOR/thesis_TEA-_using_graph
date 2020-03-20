unit unit3;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls, Grids,
  ExtCtrls, DOM, XMLWrite;

type
     link_full = object
   private
     {private declarations}
   public
     {public declarations}
     var
     id_l:integer;
     node_to_link:Double;
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
     links:array of link_full;
   end;

  { TForm3 }

  TForm3 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    Button3: TButton;
    GroupBox1: TGroupBox;
    Label1: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    LabeledEdit1: TLabeledEdit;
    LabeledEdit2: TLabeledEdit;
    PaintBox1: TPaintBox;
    StringGrid1: TStringGrid;
    procedure Button1Click(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure Button3Click(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure GroupBox1Click(Sender: TObject);
    procedure LabeledEdit1Change(Sender: TObject);
    procedure PaintBox1Paint(Sender: TObject);
  private

  public
    full_graph: array of node_full;
    links_done:array of integer;
    links_done_XML:array of integer;
  end;

var
  Form3: TForm3;
  l6:integer;
  l7:integer;
  l8:integer;
  l9:integer;
  l10:integer;
  aux8:integer;
  aux9:integer;
  aux10:integer;
  aux11:integer;
  aux12:integer;
  aux13:integer;
  aux14:integer;
  node_id:integer;
  node_x:Double;
  node_y:Double;
  node_t_id:integer;
  node_t_x:Double;
  node_t_y:Double;
  l_id:integer;
  dist:Double;
  l_done:integer;
  l_or_x:Double;
  l_or_y:Double;
  l_id_count:integer;
  R3:integer;
  R4:integer;
  R5:integer;
  R6:integer;
  R7:integer;
  R8:integer;
  n_div:Double;
  div_dist:Double;
  coord_dist_x:Double;
  coord_dist_y:Double;
  flag_grid1:integer;
  possX:string;
  possY:string;
  id_g:string;
  n_links:string;
  def_g:string;
  db1:integer;
  db2:integer;
  x_l1:integer;
  x_l2:integer;
  lxml_done:integer;
  R_XML:integer;
  X_print:Longint;
  Y_print:Longint;
  X_print1:Longint;
  Y_print1:Longint;
  n_tlink:Double;
  i_node:integer;
implementation
  uses
    unit1_Dall, unit2;
{$R *.lfm}

  { TForm3 }

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


function get_max_id_node(nodelist:a_node):integer;
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
       if i_max<i_curr then
          begin
            i_max:=i_curr;
          end;
      end;
      get_max_id_node:=i_max;
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

  procedure TForm3.Button2Click(Sender: TObject);
  begin
        {Graph contruction}
        l6:=length(form1.intersection_nodesXY);
        l8:=length(full_graph);
        R3:=l8;
        l_id_count:=1;
        if ((l6>0) and (l8<1)) then
        begin
          for aux8:=0 to l6-1 do
          begin
            node_x:=form1.intersection_nodesXY[aux8].posrealX;
            node_y:=form1.intersection_nodesXY[aux8].posrealY;
            SetLength(full_graph, R3+1);
            full_graph[R3].id:=R3+1;
            full_graph[R3].pos_X:=node_x;
            full_graph[R3].pos_Y:=node_y;
            full_graph[R3].defined:=1;
            R3:=R3+1;
          end;
          for aux8:=0 to l6-1 do
          begin
           l7:=length(form1.intersection_nodesXY[aux8].links);
           node_x:=form1.intersection_nodesXY[aux8].posrealX;
           node_y:=form1.intersection_nodesXY[aux8].posrealY;
          begin
            if l7>0 then
               begin
                 for aux9:=0 to l7-1 do
                     begin
                     l_id:=form1.intersection_nodesXY[aux8].links[aux9].id_l;
                     l8:=length(links_done);
                     l_done:=0;
                      if l8>0 then
                         begin
                           for aux10:=0 to l8-1 do
                               if l_id=links_done[aux10] then
                                  begin
                                    l_done:=1;
                                  end;
                         end;
                      if  l_done<1 then
                         begin
                             node_t_id:=get_index_node(form1.intersection_nodesXY,form1.intersection_nodesXY[aux8].links[aux9].node_to_link);
                             dist:=form1.intersection_nodesXY[aux8].links[aux9].distance;
                             node_x:=form1.intersection_nodesXY[aux8].posrealX;
                             node_y:=form1.intersection_nodesXY[aux8].posrealY;
                             node_t_x:=form1.intersection_nodesXY[node_t_id-1].posrealX;
                             node_t_Y:=form1.intersection_nodesXY[node_t_id-1].posrealY;
                             l_or_x:=node_t_x-node_x;
                             l_or_y:=node_t_y-node_y;
                             n_div:=dist/form1.vel_nom;
                             div_dist:=dist/trunc(n_div);
                             if n_div>1 then
                             begin
                             if (abs(l_or_x)>4) and  (abs(l_or_y)<4) then
                                begin
                                   coord_dist_x:=l_or_x/trunc(n_div);
                                   for aux12:=0 to trunc(n_div-1) do
                                   begin
                                        if aux12=0 then
                                           begin
                                        {Create first link node}
                                        R4:=0;
                                        R5:=length(full_graph[aux8].links);
                                        SetLength(full_graph, R3+1);
                                        full_graph[R3].id:=R3+1;
                                        node_x:=node_x+coord_dist_x;
                                        node_y:=node_y;
                                        full_graph[R3].pos_X:=node_x;
                                        full_graph[R3].pos_Y:=node_y;
                                        full_graph[R3].defined:=0;
                                        {Backwards link declaration}
                                        SetLength(full_graph[aux8].links, R5+1);
                                        full_graph[aux8].links[R5].id_l:=l_id_count;
                                        full_graph[aux8].links[R5].distance:=div_dist;
                                        full_graph[aux8].links[R5].node_to_link:=R3+1;
                                        SetLength(full_graph[R3].links, R4+1);
                                        full_graph[R3].links[R4].id_l:=l_id_count;
                                        full_graph[R3].links[R4].distance:=div_dist;
                                        full_graph[R3].links[R4].node_to_link:=aux8+1;
                                        R3:=R3+1;
                                        l_id_count:=l_id_count+1;
                                           end
                                        else if aux12<trunc(n_div-1) then
                                        begin
                                        {Create link node}
                                        R4:=0;
                                        R5:=length(full_graph[R3-1].links);
                                        SetLength(full_graph, R3+1);
                                        full_graph[R3].id:=R3+1;
                                        node_x:=node_x+coord_dist_x;
                                        node_y:=node_y;
                                        full_graph[R3].pos_X:=node_x;
                                        full_graph[R3].pos_Y:=node_y;
                                        full_graph[R3].defined:=0;
                                        {Backwards link declaration}
                                        SetLength(full_graph[R3-1].links, R5+1);
                                        full_graph[R3-1].links[R5].id_l:=l_id_count;
                                        full_graph[R3-1].links[R5].distance:=div_dist;
                                        full_graph[R3-1].links[R5].node_to_link:=R3+1;
                                        SetLength(full_graph[R3].links, R4+1);
                                        full_graph[R3].links[R4].id_l:=l_id_count;
                                        full_graph[R3].links[R4].distance:=div_dist;
                                        full_graph[R3].links[R4].node_to_link:=R3;
                                        R3:=R3+1;
                                        l_id_count:=l_id_count+1;
                                        end
                                        else
                                        begin
                                       {Create final link node}
                                        R4:=0;
                                        R8:=length(full_graph[node_t_id-1].links);
                                        R5:=length(full_graph[R3-1].links);
                                        SetLength(full_graph, R3+1);
                                        full_graph[R3].id:=R3+1;
                                        node_x:=node_x+coord_dist_x;
                                        node_y:=node_y;
                                        full_graph[R3].pos_X:=node_x;
                                        full_graph[R3].pos_Y:=node_y;
                                        full_graph[R3].defined:=0;
                                        {Backwards link declaration}
                                        SetLength(full_graph[R3-1].links, R5+1);
                                        full_graph[R3-1].links[R5].id_l:=l_id_count;
                                        full_graph[R3-1].links[R5].distance:=div_dist;
                                        full_graph[R3-1].links[R5].node_to_link:=R3+1;
                                        SetLength(full_graph[R3].links, R4+1);
                                        full_graph[R3].links[R4].id_l:=l_id_count;
                                        full_graph[R3].links[R4].distance:=div_dist;
                                        full_graph[R3].links[R4].node_to_link:=R3;
                                        l_id_count:=l_id_count+1;
                                        {Foward link declaration}
                                        SetLength(full_graph[R3].links, R4+2);
                                        full_graph[R3].links[R4+1].id_l:=l_id_count;
                                        full_graph[R3].links[R4+1].distance:=div_dist;
                                        full_graph[R3].links[R4+1].node_to_link:=node_t_id;
                                        SetLength(full_graph[node_t_id-1].links, R8+1);
                                        full_graph[node_t_id-1].links[R8].id_l:=l_id_count;
                                        full_graph[node_t_id-1].links[R8].distance:=div_dist;
                                        full_graph[node_t_id-1].links[R8].node_to_link:=R3+1;
                                        R3:=R3+1;
                                        l_id_count:=l_id_count+1;
                                        SetLength(links_done, l8+1);
                                        links_done[l8]:=l_id;

                                       end;

                                   end;
                                end
                             else if (abs(l_or_x)<4) and  (abs(l_or_y)>4) then
                                begin
                                  coord_dist_y:=l_or_y/trunc(n_div);
                                    for aux12:=0 to trunc(n_div-1) do
                                   begin
                                        if aux12=0 then
                                           begin
                                        R4:=0;
                                        R5:=length(full_graph[aux8].links);
                                        SetLength(full_graph, R3+1);
                                        full_graph[R3].id:=R3+1;
                                        node_x:=node_x;
                                        node_y:=node_y+coord_dist_y;
                                        full_graph[R3].pos_X:=node_x;
                                        full_graph[R3].pos_Y:=node_y;
                                        full_graph[R3].defined:=0;
                                         SetLength(full_graph[aux8].links, R5+1);
                                        full_graph[aux8].links[R5].id_l:=l_id_count;
                                        full_graph[aux8].links[R5].distance:=div_dist;
                                        full_graph[aux8].links[R5].node_to_link:=R3+1;
                                        SetLength(full_graph[R3].links, R4+1);
                                        full_graph[R3].links[R4].id_l:=l_id_count;
                                        full_graph[R3].links[R4].distance:=div_dist;
                                        full_graph[R3].links[R4].node_to_link:=aux8+1;
                                        R3:=R3+1;
                                        l_id_count:=l_id_count+1;
                                           end
                                        else if aux12<trunc(n_div-1) then
                                        begin
                                        R4:=0;
                                        R5:=length(full_graph[R3-1].links);
                                        SetLength(full_graph, R3+1);
                                        full_graph[R3].id:=R3+1;
                                        node_x:=node_x;
                                        node_y:=node_y+coord_dist_y;
                                        full_graph[R3].pos_X:=node_x;
                                        full_graph[R3].pos_Y:=node_y;
                                        full_graph[R3].defined:=0;
                                        SetLength(full_graph[R3-1].links, R5+1);
                                        full_graph[R3-1].links[R5].id_l:=l_id_count;
                                        full_graph[R3-1].links[R5].distance:=div_dist;
                                        full_graph[R3-1].links[R5].node_to_link:=R3+1;
                                        SetLength(full_graph[R3].links, R4+1);
                                        full_graph[R3].links[R4].id_l:=l_id_count;
                                        full_graph[R3].links[R4].distance:=div_dist;
                                        full_graph[R3].links[R4].node_to_link:=R3;
                                        full_graph[node_t_id-1].links[R4].id_l:=l_id_count;
                                        full_graph[node_t_id-1].links[R4].distance:=div_dist;
                                        full_graph[node_t_id-1].links[R4].node_to_link:=R3;
                                        R3:=R3+1;
                                        l_id_count:=l_id_count+1;
                                        end
                                        else
                                        begin
                                        {Create final link node}
                                        R4:=0;
                                        R8:=length(full_graph[node_t_id-1].links);
                                        R5:=length(full_graph[R3-1].links);
                                        SetLength(full_graph, R3+1);
                                        full_graph[R3].id:=R3+1;
                                        node_x:=node_x;
                                        node_y:=node_y+coord_dist_y;
                                        full_graph[R3].pos_X:=node_x;
                                        full_graph[R3].pos_Y:=node_y;
                                        full_graph[R3].defined:=0;
                                        {Backwards link declaration}
                                        SetLength(full_graph[R3-1].links, R5+1);
                                        full_graph[R3-1].links[R5].id_l:=l_id_count;
                                        full_graph[R3-1].links[R5].distance:=div_dist;
                                        full_graph[R3-1].links[R5].node_to_link:=R3+1;
                                        SetLength(full_graph[R3].links, R4+1);
                                        full_graph[R3].links[R4].id_l:=l_id_count;
                                        full_graph[R3].links[R4].distance:=div_dist;
                                        full_graph[R3].links[R4].node_to_link:=R3;
                                        l_id_count:=l_id_count+1;
                                        {Foward link declaration}
                                        SetLength(full_graph[R3].links, R4+2);
                                        full_graph[R3].links[R4+1].id_l:=l_id_count;
                                        full_graph[R3].links[R4+1].distance:=div_dist;
                                        full_graph[R3].links[R4+1].node_to_link:=node_t_id;
                                        SetLength(full_graph[node_t_id-1].links, R8+1);
                                        full_graph[node_t_id-1].links[R8].id_l:=l_id_count;
                                        full_graph[node_t_id-1].links[R8].distance:=div_dist;
                                        full_graph[node_t_id-1].links[R8].node_to_link:=R3+1;
                                        R3:=R3+1;
                                        l_id_count:=l_id_count+1;
                                        SetLength(links_done, l8+1);
                                        links_done[l8]:=l_id;

                                        end;


                                   end;
                                end
                             else
                                begin
                                    coord_dist_x:=l_or_x/trunc(n_div);
                                    coord_dist_y:=l_or_y/trunc(n_div);
                                     for aux12:=0 to trunc(n_div-1) do
                                     begin
                                          if aux12=0 then
                                             begin
                                              R4:=0;
                                            R5:=length(full_graph[aux8].links);
                                            SetLength(full_graph, R3+1);
                                            full_graph[R3].id:=R3+1;
                                            node_x:=node_x+coord_dist_x;
                                            node_y:=node_y+coord_dist_y;
                                            full_graph[R3].pos_X:=node_x;
                                            full_graph[R3].pos_Y:=node_y;
                                            full_graph[R3].defined:=0;
                                            SetLength(full_graph[aux8].links, R5+1);
                                            full_graph[aux8].links[R5].id_l:=l_id_count;
                                            full_graph[aux8].links[R5].distance:=div_dist;
                                            full_graph[aux8].links[R5].node_to_link:=R3+1;
                                            SetLength(full_graph[R3].links, R4+1);
                                            full_graph[R3].links[R4].id_l:=l_id_count;
                                            full_graph[R3].links[R4].distance:=div_dist;
                                            full_graph[R3].links[R4].node_to_link:=aux8+1;
                                            R3:=R3+1;
                                            l_id_count:=l_id_count+1;
                                             end
                                          else if aux12<trunc(n_div-1) then
                                          begin
                                            R4:=0;
                                            R5:=length(full_graph[R3-1].links);
                                            SetLength(full_graph, R3+1);
                                            full_graph[R3].id:=R3+1;
                                            node_x:=node_x+coord_dist_x;
                                            node_y:=node_y+coord_dist_y;
                                            full_graph[R3].pos_X:=node_x;
                                            full_graph[R3].pos_Y:=node_y;
                                            full_graph[R3].defined:=0;
                                            SetLength(full_graph[R3-1].links, R5+1);
                                            full_graph[R3-1].links[R5].id_l:=l_id_count;
                                            full_graph[R3-1].links[R5].distance:=div_dist;
                                            full_graph[R3-1].links[R5].node_to_link:=R3+1;
                                            SetLength(full_graph[R3].links, R4+1);
                                            full_graph[R3].links[R4].id_l:=l_id_count;
                                            full_graph[R3].links[R4].distance:=div_dist;
                                            full_graph[R3].links[R4].node_to_link:=R3;
                                            R3:=R3+1;
                                            l_id_count:=l_id_count+1;
                                          end
                                          else
                                          begin
                                            {Create final link node}
                                        R4:=0;
                                        R8:=length(full_graph[node_t_id-1].links);
                                        R5:=length(full_graph[R3-1].links);
                                        SetLength(full_graph, R3+1);
                                        full_graph[R3].id:=R3+1;
                                        node_x:=node_x+coord_dist_x;
                                        node_y:=node_y+coord_dist_y;
                                        full_graph[R3].pos_X:=node_x;
                                        full_graph[R3].pos_Y:=node_y;
                                        full_graph[R3].defined:=0;
                                        {Backwards link declaration}
                                        SetLength(full_graph[R3-1].links, R5+1);
                                        full_graph[R3-1].links[R5].id_l:=l_id_count;
                                        full_graph[R3-1].links[R5].distance:=div_dist;
                                        full_graph[R3-1].links[R5].node_to_link:=R3+1;
                                        SetLength(full_graph[R3].links, R4+1);
                                        full_graph[R3].links[R4].id_l:=l_id_count;
                                        full_graph[R3].links[R4].distance:=div_dist;
                                        full_graph[R3].links[R4].node_to_link:=R3;
                                        l_id_count:=l_id_count+1;
                                        {Foward link declaration}
                                        SetLength(full_graph[R3].links, R4+2);
                                        full_graph[R3].links[R4+1].id_l:=l_id_count;
                                        full_graph[R3].links[R4+1].distance:=div_dist;
                                        full_graph[R3].links[R4+1].node_to_link:=node_t_id;
                                        SetLength(full_graph[node_t_id-1].links, R8+1);
                                        full_graph[node_t_id-1].links[R8].id_l:=l_id_count;
                                        full_graph[node_t_id-1].links[R8].distance:=div_dist;
                                        full_graph[node_t_id-1].links[R8].node_to_link:=R3+1;
                                        R3:=R3+1;
                                        l_id_count:=l_id_count+1;
                                          SetLength(links_done, l8+1);
                                        links_done[l8]:=l_id;

                                          end;


                                     end;
                                end;
                             end
                             else
                             begin
                                  R8:=length(full_graph[node_t_id-1].links);
                                  R5:=length(full_graph[aux8].links);
                                  {Backwards link declaration}
                                   SetLength(full_graph[aux8].links, R5+1);
                                   full_graph[aux8].links[R5].id_l:=l_id_count;
                                   full_graph[aux8].links[R5].distance:=div_dist;
                                   full_graph[aux8].links[R5].node_to_link:=node_t_id;
                                   {Foward link declaration}
                                    SetLength(full_graph[node_t_id-1].links, R8+1);
                                    full_graph[node_t_id-1].links[R8].id_l:=l_id_count;
                                    full_graph[node_t_id-1].links[R8].distance:=div_dist;
                                    full_graph[node_t_id-1].links[R8].node_to_link:=aux8+1;
                                    l_id_count:=l_id_count+1;
                                    SetLength(links_done, l8+1);
                                    links_done[l8]:=l_id;

                             end;


                       end;
                 end;
               end;

          end;
         end;
  end;
         {Printing the graph contruction}
         l10:=length(full_graph);

    if l10>0 then
    begin
      if flag_grid1<1 then
      begin
      R7:=1;
      for aux13:=0 to l10-1 do
      begin
         possX:=FloatToStr(full_graph[aux13].pos_X);
         possY:=FloatToStr(full_graph[aux13].pos_Y);
         id_g:=FloatToStr(full_graph[aux13].id);
         n_links:=FloatToStr(length(full_graph[aux13].links));
         def_g:=FloatToStr(full_graph[aux13].defined);
         StringGrid1.InsertRowWithValues(1,[id_g, possX, possY, def_g, n_links ]);
         R7:=R7+1;
      end;
       flag_grid1:=flag_grid+1;
      end
      else
      begin
         for aux3:=1 to R7-1 do
       begin
         StringGrid1.DeleteRow(1);
      end;
         R7:=1;
        for aux13:=0 to l10-1 do
        begin
           possX:=FloatToStr(full_graph[aux13].pos_X);
         possY:=FloatToStr(full_graph[aux13].pos_Y);
         id_g:=FloatToStr(full_graph[aux13].id);
         n_links:=FloatToStr(length(full_graph[aux13].links));
         def_g:=FloatToStr(full_graph[aux13].defined);
         StringGrid1.InsertRowWithValues(1,[id_g, possX, possY, def_g, n_links ]);
         R7:=R7+1;
        end;
        flag_grid1:=flag_grid1+1;
      end;
  end;
     Label3.Caption:=FloatToStr(l6);
     Label4.Caption:=FloatToStr(length(full_graph)-l6);
     Label6.Caption:=FloatToStr(form1.vel_nom);

  Canvas:= form3.PaintBox1.Canvas;
  Canvas.Pen.Width:=8;
  Canvas.Pen.Color:=clGreen;
  l4:=length(full_graph);
  if l4>0 then
  begin
    for aux4:=0 to l4-1 do
    begin
    X_print:=round(full_graph[aux4].pos_X*815/form1.o_w);
    Y_print:=round(full_graph[aux4].pos_Y*750/form1.o_h);
     Canvas.Rectangle (X_print-1,Y_print-1,X_print+1,Y_print+1);
    end;
    Canvas.Pen.Width:=2;
    Canvas.Pen.Color:=clLime;
    for aux4:=0 to l4-1 do
    begin
     l5:=length(full_graph[aux4].links);
     X_print:=round(full_graph[aux4].pos_X*815/form1.o_w);
     Y_print:=round(full_graph[aux4].pos_Y*750/form1.o_h);
     for aux5:=0 to l5-1 do
     begin
       n_tlink:=full_graph[aux4].links[aux5].node_to_link;
       for aux6:=0 to l4-1 do
       begin
         if n_tlink=full_graph[aux6].id then
         begin
            X_print1:=round(full_graph[aux6].pos_X*815/form1.o_w);
            Y_print1:=round(full_graph[aux6].pos_Y*750/form1.o_h);
           Canvas.Line(X_print, Y_print, X_print1, Y_print1);
         end;
       end;
     end;
    end;
  end;
  end;

procedure TForm3.Button1Click(Sender: TObject);
  var
  Doc: TXMLDocument;                                  // variable to document
  RootNode, parentNode, nofilho: TDOMNode;                    // variable to nodes
begin
  try
    // Create a document
    Doc := TXMLDocument.Create;

    // Create a root node
    RootNode := Doc.CreateElement('Map');
    Doc.Appendchild(RootNode);                           // save root node

    // Create a nodes section
    RootNode:= Doc.DocumentElement;
    parentNode := Doc.CreateElement('Nodes');
    RootNode.Appendchild(parentNode);                          // save parent node
    x_l1:=length(full_graph);
    for aux13:=0 to x_l1-1 do
      begin
        // Create a node
        parentNode := Doc.CreateElement('Node');                // create a child node
        TDOMElement(parentNode).SetAttribute('Id',FloatToStr(full_graph[aux13].id) );     // create atributes
        RootNode.ChildNodes.Item[0].AppendChild(parentNode);       // insert child node in respective parent node

         // Add Coords X
        parentNode := Doc.CreateElement('x');               // create a child node
        nofilho := Doc.CreateTextNode(FloatToStr(full_graph[aux13].pos_X));               // insert a value to node
        parentNode.Appendchild(nofilho);                         // save node
        RootNode.ChildNodes.Item[0].ChildNodes.Item[aux13].AppendChild(parentNode);     // insert a childnode in respective parent node


    
         // Add Coords Y
        parentNode := Doc.CreateElement('y');               // create a child node
        nofilho := Doc.CreateTextNode(FloatToStr(full_graph[aux13].pos_Y));               // insert a value to node
        parentNode.Appendchild(nofilho);                         // save node
        RootNode.ChildNodes.Item[0].ChildNodes.Item[aux13].AppendChild(parentNode);     // insert a childnode in respective parent node


         // Add Link number
        parentNode := Doc.CreateElement('Number_of_Links');               // create a child node
        nofilho := Doc.CreateTextNode(FloatToStr(length(full_graph[aux13].links)));               // insert a value to node
        parentNode.Appendchild(nofilho);                         // save node
        RootNode.ChildNodes.Item[0].ChildNodes.Item[aux13].AppendChild(parentNode);     // insert a childnode in respective parent node

         // Add defined property
        parentNode := Doc.CreateElement('Defined');               // create a child node
        nofilho := Doc.CreateTextNode(FloatToStr(full_graph[aux13].defined));               // insert a value to node
        parentNode.Appendchild(nofilho);                         // save node
        RootNode.ChildNodes.Item[0].ChildNodes.Item[aux13].AppendChild(parentNode);     // insert a childnode in respective parent node
      end;

    parentNode := Doc.CreateElement('Links');
    RootNode.Appendchild(parentNode);                          // save parent node
    R_XML:=0;
    for aux13:=0 to x_l1-1 do
      begin
           x_l2:=length(full_graph[aux13].links);
           for aux14:=0 to x_l2-1 do
               begin
                    l8:=length(links_done_XML);
                     lxml_done:=0;
                      if l8>0 then
                         begin
                           for aux10:=0 to l8-1 do
                               if full_graph[aux13].Links[aux14].id_l=links_done_XML[aux10] then
                                  begin
                                    lxml_done:=1;
                                  end;
                           end;
            if lxml_done<1 then
                begin
                  // Create a Link
                  parentNode := Doc.CreateElement('Link');                // create a child node
                  TDOMElement(parentNode).SetAttribute('Id',FloatToStr(full_graph[aux13].Links[aux14].id_l) );     // create atributes
                  RootNode.ChildNodes.Item[1].AppendChild(parentNode);       // insert child node in respective parent node

                  // Add Node1
                  parentNode := Doc.CreateElement('Node1_Id');               // create a child node
                  nofilho := Doc.CreateTextNode(FloatToStr(full_graph[aux13].id));               // insert a value to node
                  parentNode.Appendchild(nofilho);                         // save node
                  RootNode.ChildNodes.Item[1].ChildNodes.Item[R_XML].AppendChild(parentNode);     // insert a childnode in respective parent node

                  // Add Node2
                  parentNode := Doc.CreateElement('Node2_Id');               // create a child node
                  nofilho := Doc.CreateTextNode(FloatToStr(full_graph[aux13].links[aux14].node_to_link));               // insert a value to node
                  parentNode.Appendchild(nofilho);                         // save node
                  RootNode.ChildNodes.Item[1].ChildNodes.Item[R_XML].AppendChild(parentNode);     // insert a childnode in respective parent node

                  // Add Distance
                  parentNode := Doc.CreateElement('Distance');               // create a child node
                  nofilho := Doc.CreateTextNode(FloatToStr(full_graph[aux13].links[aux14].distance));               // insert a value to node
                  parentNode.Appendchild(nofilho);                         // save node
                  RootNode.ChildNodes.Item[1].ChildNodes.Item[R_XML].AppendChild(parentNode);     // insert a childnode in respective parent node

                  SetLength(links_done_XML, l8+1);
                  links_done_XML[l8]:=full_graph[aux13].Links[aux14].id_l;
                  R_XML:=R_XML+1;
                end;
               end;

           end;

          writeXMLFile(Doc, 'test.xml');                     // write to XML
  finally
    Doc.Free;                                          // free memory
  end;
end;

procedure TForm3.Button3Click(Sender: TObject);
begin
  db1:=StrToInt(labelededit1.Text);
  db2:=StrToInt(labelededit2.Text);
  label7.Caption:=FloatToStr(full_graph[db1-1].links[db2-1].node_to_link);
end;

procedure TForm3.FormCreate(Sender: TObject);
begin
  flag_grid1:=0;
end;

procedure TForm3.GroupBox1Click(Sender: TObject);
begin

end;

procedure TForm3.LabeledEdit1Change(Sender: TObject);
begin

end;

procedure TForm3.PaintBox1Paint(Sender: TObject);
begin
    Canvas:= form3.PaintBox1.Canvas;
  Canvas.Pen.Width:=8;
  Canvas.Pen.Color:=clGreen;
  l4:=length(full_graph);
  if l4>0 then
  begin
    for aux4:=0 to l4-1 do
    begin
    X_print:=round(full_graph[aux4].pos_X*815/form1.o_w);
    Y_print:=round(full_graph[aux4].pos_Y*750/form1.o_h);
     Canvas.Rectangle (X_print-1,Y_print-1,X_print+1,Y_print+1);
    end;
    Canvas.Pen.Width:=2;
    Canvas.Pen.Color:=clLime;
    for aux4:=0 to l4-1 do
    begin
     l5:=length(full_graph[aux4].links);
     X_print:=round(full_graph[aux4].pos_X*815/form1.o_w);
     Y_print:=round(full_graph[aux4].pos_Y*750/form1.o_h);
     for aux5:=0 to l5-1 do
     begin
       n_tlink:=full_graph[aux4].links[aux5].node_to_link;
       for aux6:=0 to l4-1 do
       begin
         if n_tlink=full_graph[aux6].id then
         begin
            X_print1:=round(full_graph[aux6].pos_X*815/form1.o_w);
            Y_print1:=round(full_graph[aux6].pos_Y*750/form1.o_h);
           Canvas.Line(X_print, Y_print, X_print1, Y_print1);
         end;
       end;
     end;
    end;
  end;
end;

end.

