unit Unit4;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, Grids, StdCtrls,
  ExtCtrls;

type

  { TForm4 }

  TForm4 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    Button3: TButton;
    Button4: TButton;
    PaintBox1: TPaintBox;
    StringGrid1: TStringGrid;
    procedure Button1Click(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure Button3Click(Sender: TObject);
    procedure Button4Click(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure PaintBox1Paint(Sender: TObject);
  private

  public
     Check_nodes:array of integer;
  end;

var
  Form4: TForm4;
  l1:integer;
  l2:integer;
  l3:integer;
  aux1:integer;
  aux2:integer;
  aux3:integer;
  count:integer;
  R1:integer;
  possX:string;
  possY:string;
  id_g:string;
  n_links:string;
  cr_g:string;
  flag_grid1:integer;
  flag_stop:integer;
  X_print:LongInt;
  Y_print:LongInt;
  X_print1:LongInt;
  Y_print1:LongInt;
  id_l:integer;
  c1_print:integer;
  c2_print:integer;
implementation
   uses
    unit1_Dall, unit2,unit3;
{$R *.lfm}

{ TForm4 }

procedure TForm4.Button1Click(Sender: TObject);
begin
      l1:=length(form1.intersection_nodesXY);
      flag_stop:=0;
      for aux1:=0 to l1-1 do
      begin
          l2:=length(form1.intersection_nodesXY[aux1].links);
          if l2<2 then
          begin
              form1.intersection_nodesXY[aux1].iscritical:=1;
              R1:=length(Check_nodes);
              SetLength(Check_nodes, R1+1);
              Check_nodes[R1]:=form1.intersection_nodesXY[aux1].id;
              flag_stop:=1;
          end;
      end;
      while flag_stop=1 do
      begin
      flag_stop:=0;
      for aux1:=0 to l1-1 do
      begin
          if form1.intersection_nodesXY[aux1].iscritical<1 then
          begin
            l2:=length(form1.intersection_nodesXY[aux1].links);
            count:=0;
            for aux2:=0 to l2-1 do
            begin
                l3:=length(Check_nodes);
                for aux3:=0 to l3-1 do
                if Check_nodes[aux3]=form1.intersection_nodesXY[aux1].links[aux2].node_to_link then
                begin
                  count:=count+1;
                end;
            end;
            l4:=l2-count;
            if l4<2 then
               begin
                    form1.intersection_nodesXY[aux1].iscritical:=2;
                    R1:=length(Check_nodes);
                    SetLength(Check_nodes, R1+1);
                    Check_nodes[R1]:=form1.intersection_nodesXY[aux1].id;
                    flag_stop:=1;
               end;
          end;

      end;
      end;

    //Print Grid
  if l1>0 then
    begin
      if flag_grid1<1 then
      begin
      for aux1:=0 to l1-1 do
      begin
         possX:=FloatToStr( form1.intersection_nodesXY[aux1].posrealx);
         possY:=FloatToStr(form1.intersection_nodesXY[aux1].posrealy);
         id_g:=FloatToStr(form1.intersection_nodesXY[aux1].id);
         n_links:=FloatToStr(length(form1.intersection_nodesXY[aux1].links));
         cr_g:=FloatToStr(form1.intersection_nodesXY[aux1].iscritical);
         StringGrid1.InsertRowWithValues(1,[id_g, possX, possY, n_links, cr_g ]);
      end;
       flag_grid1:=flag_grid+1;
      end
      else
      begin
         for aux1:=0 to l1-1 do
       begin
         StringGrid1.DeleteRow(1);
      end;
        for aux1:=0 to l1-1 do
        begin
          possX:=FloatToStr( form1.intersection_nodesXY[aux1].posrealx);
         possY:=FloatToStr(form1.intersection_nodesXY[aux1].posrealy);
         id_g:=FloatToStr(form1.intersection_nodesXY[aux1].id);
         n_links:=FloatToStr(length(form1.intersection_nodesXY[aux1].links));
         cr_g:=FloatToStr(form1.intersection_nodesXY[aux1].iscritical);
         StringGrid1.InsertRowWithValues(1,[id_g, possX, possY, n_links, cr_g ]);
        end;
        flag_grid1:=flag_grid1+1;
      end;
  end;


end;

procedure TForm4.Button2Click(Sender: TObject);
begin
     Form3.Show;
     Form4.hide;
end;

procedure TForm4.Button3Click(Sender: TObject);
begin
  l1:=length(form1.intersection_nodesXY);
  SetLength(form1.intersection_nodesXY, 0);
   for aux3:=1 to l1-1 do
   begin
     StringGrid1.DeleteRow(1);
  end;
   R:=0;
   Form1.Show;
   Form4.hide;
end;

procedure TForm4.Button4Click(Sender: TObject);
begin
    Canvas := form4.PaintBox1.Canvas;
    l1:=length(form1.intersection_nodesXY);

    //Print Links
    Canvas.Pen.Width:=2;
    Canvas.Pen.Color:=clLime;
    for aux1:=0 to l1-1 do
        begin
          X_print:=form1.intersection_nodesXY[aux1].posnX;
          Y_print:=form1.intersection_nodesXY[aux1].posnY;
          l2:=length(form1.intersection_nodesXY[aux1].links);
          c1_print:=form1.intersection_nodesXY[aux1].iscritical;
           for aux2:=0 to l2-1 do
           begin
              id_l:=form1.intersection_nodesXY[aux1].links[aux2].node_to_link;
               for aux3:=0 to l1-1 do
                   begin
                     if form1.intersection_nodesXY[aux3].id=id_l then
                     begin
                        X_print1:=form1.intersection_nodesXY[aux3].posnX;
                        Y_print1:=form1.intersection_nodesXY[aux3].posnY;
                        c2_print:=form1.intersection_nodesXY[aux3].iscritical;
                        if ((c1_print=1) or (c2_print=1)) then
                        begin
                           Canvas.Pen.Color:=clRed;
                        end
                        else if ((c1_print=2) or (c2_print=2)) then
                        begin
                            Canvas.Pen.Color:=clblue;
                         end
                         else
                         begin
                             Canvas.Pen.Color:=clLime;
                         end;

                        Canvas.Line(X_print, Y_print, X_print1, Y_print1);
                     end;
                   end;
           end;
        end;
    //Print Nodes
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

procedure TForm4.FormCreate(Sender: TObject);
begin
  flag_grid1:=0;
end;

procedure TForm4.FormShow(Sender: TObject);
begin
  l1:=length(form1.intersection_nodesXY);
      flag_stop:=0;
      for aux1:=0 to l1-1 do
      begin
          l2:=length(form1.intersection_nodesXY[aux1].links);
          if l2<2 then
          begin
              form1.intersection_nodesXY[aux1].iscritical:=1;
              R1:=length(Check_nodes);
              SetLength(Check_nodes, R1+1);
              Check_nodes[R1]:=form1.intersection_nodesXY[aux1].id;
              flag_stop:=1;
          end;
      end;
      while flag_stop=1 do
      begin
      flag_stop:=0;
      for aux1:=0 to l1-1 do
      begin
          if form1.intersection_nodesXY[aux1].iscritical<1 then
          begin
            l2:=length(form1.intersection_nodesXY[aux1].links);
            count:=0;
            for aux2:=0 to l2-1 do
            begin
                l3:=length(Check_nodes);
                for aux3:=0 to l3-1 do
                if Check_nodes[aux3]=form1.intersection_nodesXY[aux1].links[aux2].node_to_link then
                begin
                  count:=count+1;
                end;
            end;
            l4:=l2-count;
            if l4<2 then
               begin
                    form1.intersection_nodesXY[aux1].iscritical:=2;
                    R1:=length(Check_nodes);
                    SetLength(Check_nodes, R1+1);
                    Check_nodes[R1]:=form1.intersection_nodesXY[aux1].id;
                    flag_stop:=1;
               end;
          end;

      end;
      end;

    //Print Grid
  if l1>0 then
    begin
      if flag_grid1<1 then
      begin
      for aux1:=0 to l1-1 do
      begin
         possX:=FloatToStr( form1.intersection_nodesXY[aux1].posrealx);
         possY:=FloatToStr(form1.intersection_nodesXY[aux1].posrealy);
         id_g:=FloatToStr(form1.intersection_nodesXY[aux1].id);
         n_links:=FloatToStr(length(form1.intersection_nodesXY[aux1].links));
         cr_g:=FloatToStr(form1.intersection_nodesXY[aux1].iscritical);
         StringGrid1.InsertRowWithValues(1,[id_g, possX, possY, n_links, cr_g ]);
      end;
       flag_grid1:=flag_grid+1;
      end
      else
      begin
         for aux1:=0 to l1-1 do
       begin
         StringGrid1.DeleteRow(1);
      end;
        for aux1:=0 to l1-1 do
        begin
          possX:=FloatToStr( form1.intersection_nodesXY[aux1].posrealx);
         possY:=FloatToStr(form1.intersection_nodesXY[aux1].posrealy);
         id_g:=FloatToStr(form1.intersection_nodesXY[aux1].id);
         n_links:=FloatToStr(length(form1.intersection_nodesXY[aux1].links));
         cr_g:=FloatToStr(form1.intersection_nodesXY[aux1].iscritical);
         StringGrid1.InsertRowWithValues(1,[id_g, possX, possY, n_links, cr_g ]);
        end;
        flag_grid1:=flag_grid1+1;
      end;
  end;
end;

procedure TForm4.PaintBox1Paint(Sender: TObject);
begin
      Canvas := form4.PaintBox1.Canvas;
    l1:=length(form1.intersection_nodesXY);

    //Print Links
    Canvas.Pen.Width:=2;
    Canvas.Pen.Color:=clLime;
    for aux1:=0 to l1-1 do
        begin
          X_print:=form1.intersection_nodesXY[aux1].posnX;
          Y_print:=form1.intersection_nodesXY[aux1].posnY;
          l2:=length(form1.intersection_nodesXY[aux1].links);
          c1_print:=form1.intersection_nodesXY[aux1].iscritical;
           for aux2:=0 to l2-1 do
           begin
              id_l:=form1.intersection_nodesXY[aux1].links[aux2].node_to_link;
               for aux3:=0 to l1-1 do
                   begin
                     if form1.intersection_nodesXY[aux3].id=id_l then
                     begin
                        X_print1:=form1.intersection_nodesXY[aux3].posnX;
                        Y_print1:=form1.intersection_nodesXY[aux3].posnY;
                        c2_print:=form1.intersection_nodesXY[aux3].iscritical;
                        if ((c1_print=1) or (c2_print=1)) then
                        begin
                           Canvas.Pen.Color:=clRed;
                        end
                        else if ((c1_print=2) or (c2_print=2)) then
                        begin
                            Canvas.Pen.Color:=clblue;
                         end
                         else
                         begin
                             Canvas.Pen.Color:=clLime;
                         end;

                        Canvas.Line(X_print, Y_print, X_print1, Y_print1);
                     end;
                   end;
           end;
        end;
    //Print Nodes
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

end.

