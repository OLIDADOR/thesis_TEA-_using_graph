unit Unit1;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls, Grids, Dom,
  XmlRead;

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

      a_node=array of node_full;



  { TForm1 }

  TForm1 = class(TForm)
    Label1: TLabel;
    StringGrid1: TStringGrid;
    StringGrid2: TStringGrid;
    procedure FormShow(Sender: TObject);
  private

  public

  end;

var
  Form1: TForm1;
  full_nodelist:array of node_full;
  l1:integer;
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

implementation

{$R *.lfm}

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
         StringGrid2.InsertRowWithValues(1,[inttostr(i_curr), floattostr(x), floattostr(Y)]);
        end;
      end;
      ReadXMLFile(Doc, 'C:\Users\diogo\Desktop\pascal\read_xml\test.xml');
      Nodes:= Doc.GetElementsByTagName('Node');
      Node:= Nodes[0];
      id:=Node.Attributes.Item[0];
      LABEL1.Caption:=id.NodeValue;
end;


end.

