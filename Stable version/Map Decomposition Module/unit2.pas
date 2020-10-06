unit unit2;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, ValEdit, ExtCtrls,
  ActnList, Grids, StdCtrls, fpspreadsheetgrid, fpspreadsheetctrls, RTTICtrls,unit1_Dall,unit3,unit4,unit1;

type

  { TForm2 }
  a_node=array of node;
  TForm2 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    Button3: TButton;
    Button4: TButton;
    Button6: TButton;
    GroupBox1: TGroupBox;
    Label1: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    LabeledEdit1: TLabeledEdit;
    LabeledEdit2: TLabeledEdit;
    LabeledEdit4: TLabeledEdit;
    LabeledEdit5: TLabeledEdit;
    PaintBox1: TPaintBox;
    RadioButton1: TRadioButton;
    RadioButton2: TRadioButton;
    RadioButton3: TRadioButton;
    RadioButton4: TRadioButton;
    RadioGroup1: TRadioGroup;
    RadioGroup2: TRadioGroup;
    StringGrid1: TStringGrid;
    StringGrid2: TStringGrid;

    procedure Button1Click(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure Button3Click(Sender: TObject);
    procedure Button4Click(Sender: TObject);
    procedure Button5Click(Sender: TObject);
    procedure Button6Click(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormPaint(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure LabeledEdit2Change(Sender: TObject);
    procedure LabeledEdit3Change(Sender: TObject);
    procedure PaintBox1Paint(Sender: TObject);
    procedure ValueListEditor1Click(Sender: TObject);

  private

  public
    G_report:integer;
    A_nodes:integer;
  end;

var
  Form2: TForm2;
  lc2:integer;
  lc3:integer;
  aux3:integer;
  aux4:integer;
  aux5:integer;
  aux6:integer;
  id:string;
  id_i:integer;
  newlabel: TLabel;
  flag_nld:integer;
  n_links:string;
  possX:string;
  possY:string;
  R:integer;
  R2:integer;
  R3:integer;
  Node1_id_string:string;
  Node1_id:integer;
  Node2_id_string:string;
  Node2_id:integer;
  dist_string:string;
  link_id_string:string;
  n1_int:integer;
  n2_int:integer;
  l1:integer;
  l4:integer;
  l5:integer;
  n_curr:integer;
  n_tlink:integer;
  flag_grid:integer;
  X_print:LongInt;
  Y_print:LongInt;
  X_print1:LongInt;
  Y_print1:LongInt;
  d_l_id:integer;
  l_curr:integer;
  rc:integer;
  slack:integer;
  l_done:array of integer;
  f1:integer;
  rc1:integer;
  rc2:integer;
implementation

{$R *.lfm}
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
      //Calcultes the distance between 2 nodes when the node id is given as an integer
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
  //checks if a certain element exist inside an array and returns the number of hits
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



 function Get_Distance(n1:string;n2:string; nodelist:a_node):Double;
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
    //Calcultes the distance between 2 nodes when the node id is given as a string
      n1_int:=StrtoInt(n1);
      n2_int:=StrtoInt(n2);
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
   Get_Distance:=sqrt(aux1);
   //Get_Distance:=d_x;
end;
end;
end;

function Create_link(id:integer;n2:string;d:string):link;
  var
    n2_int:integer;
    d_int:Double;
    l_temp:link;
  begin
      //Creates a link between the 2 nodes
      n2_int:=StrtoInt(n2);
      d_int:=StrtoFloat(d);
      l_temp.id_l:=id;
      l_temp.node_to_link:=n2_int;
      l_temp.distance:=d_int;
      Create_link:=l_temp;
end;

procedure Ajust_nodes (sl:integer;nodelist:a_node);

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
    n_done:array of integer;
    l_done:array of integer;
begin
  //Adjust the node positions based on a determine slack
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
       n_link:=nodelist[aux4].links[aux5].node_to_link;
       if check_existance(l_curr,l_done)<1 then
       begin
        if check_existance(n_link,n_done)<1 then
         begin
         for aux6:=0 to l4-1 do
         begin
           if nodelist[aux6].id=n_link then
           begin
              if ((nodelist[aux6].posnX<nodelist[aux4].posnX+sl) and (nodelist[aux6].posnX>nodelist[aux4].posnX-sl))  then
              begin
                  nodelist[aux6].posnX:=nodelist[aux4].posnX;
                  nodelist[aux6].posrealX:=nodelist[aux4].posrealX;
                  l6:=length(n_done);
                  setlength(n_done,l6+1);
                  n_done[l6]:=nodelist[aux6].id;
              end;
              if ((nodelist[aux6].posnY<nodelist[aux4].posnY+sl) and (nodelist[aux6].posnY>nodelist[aux4].posnY-sl))  then
              begin
                  nodelist[aux6].posnY:=nodelist[aux4].posnY;
                  nodelist[aux6].posrealY:=nodelist[aux4].posrealY;
                  l6:=length(n_done);
                  setlength(n_done,l6+1);
                  n_done[l6]:=nodelist[aux6].id;
              end;
              l7:=length(l_done);
              setlength(l_done,l7+1);
              l_done[l7]:=l_curr;
           end;
         end;
       end;
     end;
     end;
  end;
  Recalc_Dist(nodelist);
end;
end;


//procedure Add_link(n1:string;link:link;nodelist:array of node);
//
//  var
//    n1_int:integer;
//    l4:integer;
//    l5:integer;
//    aux4:integer;
//    n_curr:integer;
//
//begin
//  l4:=length(nodelist);
//  n1_int:=StrtoInt(n1);
//  if l4>0 then
//  begin
//
//  for aux4:=0 to l4-1 do
//  begin
//     n_curr:=nodelist[aux4].id;
//     if n_curr=n1_int then
//     begin
//       l5:=length(nodelist[aux4].links);
//       SetLength(nodelist[aux4].links, l5+1);
//       nodelist[aux4].links[l5].id_l:=link.id_l;
//       nodelist[aux4].links[l5].distance:=link.distance;
//       nodelist[aux4].links[l5].node_to_link:=link.node_to_link;
//     end;
//  end;
//end;
//end;




procedure TForm2.FormCreate(Sender: TObject);
begin
  R2:=1;
  flag_grid:=0;
  flag_nld:=0;
end;

procedure TForm2.FormPaint(Sender: TObject);
begin

end;

procedure TForm2.FormShow(Sender: TObject);
begin
  rc1:=StringGrid1.RowCount;
  for aux3:=1 to rc1-1 do
     begin
     StringGrid1.DeleteRow(1);
     end;

  rc2:=StringGrid2.RowCount;
  for aux3:=1 to rc2-1 do
     begin
     StringGrid2.DeleteRow(1);
     end;
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
     StringGrid2.InsertRowWithValues(R,[id, possX, possY, n_links ]);
     R:=R+1;
  end;
  flag_grid:=flag_grid+1;
  end
  else
  begin
     for aux3:=1 to R-1 do
   begin
     StringGrid2.DeleteRow(1);
  end;
     R:=1;
    for aux3:=0 to lc2-1 do
    begin
       possX:=FloatToStr(form1.intersection_nodesXY[aux3].posrealX);
       possY:=FloatToStr(form1.intersection_nodesXY[aux3].posrealY);
       id:=FloatToStr(form1.intersection_nodesXY[aux3].id);
       n_links:=FloatToStr(length(form1.intersection_nodesXY[aux3].links));
       StringGrid2.InsertRowWithValues(R,[id, possX, possY, n_links ]);
       R:=R+1;
    end;
    flag_grid:=flag_grid+1;
  end;
  end;
   setlength(l_done,0);
   l4:=length(Form1.intersection_nodesXY);
   R3:=1;
       for aux4:=0 to l4-1 do
        begin
           n_curr:=form1.intersection_nodesXY[aux4].id;
           l5:=length(form1.intersection_nodesXY[aux4].links);

           for aux5:=0 to l5-1 do
           begin
              f1:=0;
              l6:=length(l_done);
              for aux6:=0 to l6-1 do
              begin
                if l_done[aux6]=form1.intersection_nodesXY[aux4].links[aux5].id_l then
                begin
                f1:=1;
                end;
              end;
                if f1=0 then
                begin
                    Node1_id_string:=inttostr(form1.intersection_nodesXY[aux4].id);
                    Node2_id_string:=inttostr(form1.intersection_nodesXY[aux4].links[aux5].node_to_link);
                    dist_string:=floattostr(form1.intersection_nodesXY[aux4].links[aux5].distance);
                    link_id_string:= inttostr(form1.intersection_nodesXY[aux4].links[aux5].id_l);
                    StringGrid1.InsertRowWithValues(R3,[link_id_string,Node1_id_string, Node2_id_string, dist_string]);
                    l6:=length(l_done);
                    setlength(l_done,l6+1);
                    l_done[l6]:=form1.intersection_nodesXY[aux4].links[aux5].id_l;
                    R3:=R3+1;
                end;
             end;
           end;
        end;


procedure TForm2.LabeledEdit2Change(Sender: TObject);
begin

end;

procedure TForm2.LabeledEdit3Change(Sender: TObject);
begin

end;
procedure TForm2.PaintBox1Paint(Sender: TObject);
begin
  Canvas:= form2.PaintBox1.Canvas;
  Canvas.Pen.Width:=8;
  Canvas.Pen.Color:=clGreen;
  l4:=length(form1.intersection_nodesXY);
  if l4>0 then
  begin
    for aux4:=0 to l4-1 do
    begin
     id_i:=form1.intersection_nodesXY[aux4].id;
     X_print:=form1.intersection_nodesXY[aux4].posnX;
     Y_print:=form1.intersection_nodesXY[aux4].posnY;
     if flag_nld=0 then
     begin
     newlabel := TLabel.Create(Form2);
     newlabel.name := 'nlad'+inttostr(id_i);
     newlabel.caption := inttostr(id_i);
     newlabel.left := X_print+953-15;
     newlabel.Top := y_print+48-18;
     newlabel.visible := true;
     newlabel.parent := Form2;
     end;
     Canvas.Rectangle (X_print-1,Y_print-1,X_print+1,Y_print+1);
    end;
    flag_nld:=1;
    Canvas.Pen.Width:=2;
    Canvas.Pen.Color:=clLime;
    for aux4:=0 to l4-1 do
    begin
     l5:=length(form1.intersection_nodesXY[aux4].links);
     X_print:=form1.intersection_nodesXY[aux4].posnX;
     Y_print:=form1.intersection_nodesXY[aux4].posnY;
     for aux5:=0 to l5-1 do
     begin
       n_tlink:=form1.intersection_nodesXY[aux4].links[aux5].node_to_link;
       for aux6:=0 to l4-1 do
       begin
         if n_tlink=form1.intersection_nodesXY[aux6].id then
         begin
           X_print1:=form1.intersection_nodesXY[aux6].posnX;
           Y_print1:=form1.intersection_nodesXY[aux6].posnY;
           Canvas.Line(X_print, Y_print, X_print1, Y_print1);
         end;
       end;
     end;
    end;
  end;
end;


procedure TForm2.Button1Click(Sender: TObject);
begin
  if (((RadioButton1.Checked= true) or (RadioButton2.Checked= true)) and ((RadioButton3.Checked= true) or (RadioButton4.Checked= true))) then
  begin
  if RadioButton1.Checked= true then
     begin
        A_nodes:=1;
     end
  else
  begin
      A_nodes:=0;
  end;
  if RadioButton3.Checked= true then
     begin
        G_report:=1;
     end
  else
  begin
      G_report:=0;
  end;

  if (A_nodes=1) then
     begin
     if form2.LabeledEdit5.Text='' then
        begin
         label3.Caption:='Please specify the desired Slack';
        end
     else
     begin
     slack:=strtoint(form2.LabeledEdit5.Text);
     Ajust_nodes(slack,form1.intersection_nodesXY);
     end;
     end;
  if (G_report<>1) then
     begin
         Form3.Show;
         Form2.close;
     end
   else if(G_report=1) then
     begin
         Form4.Show;
         Form2.close;
     end;
  end
  else
  begin
     label3.Caption:='It is required to specify wich operations are to be executed by utilizing the radio buttons!';
  end;
end;

procedure TForm2.Button2Click(Sender: TObject);
begin
   SetLength(form1.intersection_nodesXY, 0);
   for aux3:=1 to R-1 do
   begin
     StringGrid2.DeleteRow(1);
  end;
   R:=0;
   Form1.Show;
   Form2.Close;
end;

procedure TForm2.Button3Click(Sender: TObject);
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
     StringGrid2.InsertRowWithValues(R,[id, possX, possY, n_links ]);
     R:=R+1;
  end;
  flag_grid:=flag_grid+1;
  end
  else
  begin
     for aux3:=1 to R-1 do
   begin
     StringGrid2.DeleteRow(1);
  end;
     R:=1;
    for aux3:=0 to lc2-1 do
    begin
       possX:=FloatToStr(form1.intersection_nodesXY[aux3].posrealX);
       possY:=FloatToStr(form1.intersection_nodesXY[aux3].posrealY);
       id:=FloatToStr(form1.intersection_nodesXY[aux3].id);
       n_links:=FloatToStr(length(form1.intersection_nodesXY[aux3].links));
       StringGrid2.InsertRowWithValues(R,[id, possX, possY, n_links ]);
       R:=R+1;
    end;
    flag_grid:=flag_grid+1;
  end;
  end;
end;


procedure TForm2.Button4Click(Sender: TObject);
var
  link1:link;
  link2:link;
begin
  Node1_id_string:=LabeledEdit1.text;
  Node2_id_string:=LabeledEdit2.text;
  dist_string:=FloatToStr(Get_Distance(Node1_id_string,Node2_id_string,form1.intersection_nodesXY));
  R2:=get_max_id_link(form1.intersection_nodesXY)+1;
  link_id_string:= FloatToStr(R2);
  rc:=StringGrid1.RowCount;
  StringGrid1.InsertRowWithValues(rc,[link_id_string,Node1_id_string, Node2_id_string, dist_string]);
  link1:=Create_link(R2,Node2_id_string,dist_string);
  link2:=Create_link(R2,Node1_id_string,dist_string);
  //Label3.Caption:=FloatToStr(link1.id_l);
  //Add_link(Node1_id_string,link1,form1.intersection_nodesXY)
  l4:=length(form1.intersection_nodesXY);
  n1_int:=StrtoInt(Node1_id_string);
  n2_int:=StrtoInt(Node2_id_string);
  //PRINT Link
  l1:=length(form1.intersection_nodesXY);
  Canvas.Pen.Width:=2;
  Canvas.Pen.Color:=clLime;
  for aux1:=0 to l1-1 do
      begin
        if n1_int=form1.intersection_nodesXY[aux1].id then
        begin
         X_print:=form1.intersection_nodesXY[aux1].posnX;
         Y_print:=form1.intersection_nodesXY[aux1].posnY;
        end
        else if n2_int=form1.intersection_nodesXY[aux1].id then
        begin
         X_print1:=form1.intersection_nodesXY[aux1].posnX;
         Y_print1:=form1.intersection_nodesXY[aux1].posnY;
        end;

      end;
  Canvas.Line(X_print, Y_print, X_print1, Y_print1);

 //Save Link
  if l4>0 then
  begin

  for aux4:=0 to l4-1 do
  begin
     n_curr:=form1.intersection_nodesXY[aux4].id;
     if n_curr=n1_int then
     begin
       l5:=length(form1.intersection_nodesXY[aux4].links);
       SetLength(form1.intersection_nodesXY[aux4].links, l5+1);
       form1.intersection_nodesXY[aux4].links[l5].id_l:=link1.id_l;
       form1.intersection_nodesXY[aux4].links[l5].distance:=link1.distance;
       form1.intersection_nodesXY[aux4].links[l5].node_to_link:=link1.node_to_link;
     end;
  end;
end;

  //Add_link(Node2_id_string,link2,form1.intersection_nodesXY);
  l4:=length(form1.intersection_nodesXY);
  n1_int:=StrtoInt(Node2_id_string);
  if l4>0 then
  begin

  for aux4:=0 to l4-1 do
  begin
     n_curr:=form1.intersection_nodesXY[aux4].id;
     if n_curr=n1_int then
     begin
       l5:=length(form1.intersection_nodesXY[aux4].links);
       SetLength(form1.intersection_nodesXY[aux4].links, l5+1);
       form1.intersection_nodesXY[aux4].links[l5]:=link2;
     end;
  end;
end;
  R2:=R2+1;
end;

procedure TForm2.Button5Click(Sender: TObject);
begin
     Canvas := form2.PaintBox1.Canvas;
     l1:=length(form1.intersection_nodesXY);
     Canvas.Pen.Width:=8;
     Canvas.Pen.Color:=clRed;
    for aux1:=0 to l1-1 do
        begin
          X_print:=form1.intersection_nodesXY[aux1].posnX;
          Y_print:=form1.intersection_nodesXY[aux1].posnY;
          if form1.intersection_nodesXY[aux1].iscritical=1 then
          begin
             Canvas.Pen.Color:=clRed;
          end
          else if form1.intersection_nodesXY[aux1].iscritical=2 then
          begin
             Canvas.Pen.Color:=clblue;
          end
          else
          begin
            Canvas.Pen.Color:=clGreen;
          end;

          Canvas.Rectangle (X_print-1,Y_print-1,X_print+1,Y_print+1);

        end;
end;

procedure TForm2.Button6Click(Sender: TObject);
begin
   d_l_id:=StrToInt(LabeledEdit4.Text);
   l4:=length(form1.intersection_nodesXY);
  if l4>0 then
  begin

  for aux4:=0 to l4-1 do
  begin
     n_curr:=form1.intersection_nodesXY[aux4].id;
     l5:=length(form1.intersection_nodesXY[aux4].links);
     for aux5:=0 to l5-1 do
     begin
       l_curr:=form1.intersection_nodesXY[aux4].links[aux5].id_l;
       if l_curr=d_l_id then
       begin
        for aux6:= aux5 to l5-2 do
           begin
           form1.intersection_nodesXY[aux4].links[aux6]:=form1.intersection_nodesXY[aux4].links[aux6+1];
           end;
       SetLength(form1.intersection_nodesXY[aux4].links, l5-1);
       end;
       end;
     end;
       rc:=StringGrid1.RowCount;
       for aux3:=1 to rc-1 do
         begin
           StringGrid1.DeleteRow(1);
        end;
       R3:=0;
       setlength(l_done,0);
       for aux4:=0 to l4-1 do
        begin
           n_curr:=form1.intersection_nodesXY[aux4].id;
           l5:=length(form1.intersection_nodesXY[aux4].links);

           for aux5:=0 to l5-1 do
           begin
              f1:=0;
              l6:=length(l_done);
              for aux6:=0 to l6-1 do
              begin
                if l_done[aux6]=form1.intersection_nodesXY[aux4].links[aux5].id_l then
                begin
                f1:=1;
                end;
              end;
                if f1=0 then
                begin
                    Node1_id_string:=inttostr(form1.intersection_nodesXY[aux4].id);
                    Node2_id_string:=inttostr(form1.intersection_nodesXY[aux4].links[aux5].node_to_link);
                    dist_string:=floattostr(form1.intersection_nodesXY[aux4].links[aux5].distance);
                    link_id_string:= inttostr(form1.intersection_nodesXY[aux4].links[aux5].id_l);
                    StringGrid1.InsertRowWithValues(1,[link_id_string,Node1_id_string, Node2_id_string, dist_string]);
                    l6:=length(l_done);
                    setlength(l_done,l6+1);
                    l_done[l6]:=form1.intersection_nodesXY[aux4].links[aux5].id_l;
                    R3:=R3+1;
                end;
             end;
           end;
        end;
        Invalidate;
       end;


procedure TForm2.ValueListEditor1Click(Sender: TObject);
begin

end;

end.

