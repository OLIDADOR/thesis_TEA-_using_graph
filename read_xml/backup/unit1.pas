unit Unit1;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs,Dom, XmlRead;

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
     number_of_links:integer;
     links:array of link_full;
   end;

      a_node=array of node_full;



  TForm1 = class(TForm)
  private

  public

  end;

var
  Form1: TForm1;

implementation

{$R *.lfm}

procedure read_xml(nodelist:a_node);
var
  Doc: TXMLDocument;
  Nodes: TDOMNodeList;
  Node: TDOMNode;
  id: TDOMNode;
  x: TDOMNode;
  y: TDOMNode;
  def:TDOMNode;
  nl:TDOMNode;
  Gender: TDOMNode;
  i: integer;
  id_value:string;
  x_value:string;
  y_value:string;
  def_value:string;
  nl_value:string;
  l1:integer;
begin

  ReadXMLFile(Doc, 'D:\A.xml');
  Nodes:= Doc.GetElementsByTagName('Node');
  for i:= 0 to Nodes.Count - 1 do
  begin
     Node:= Nodes[i];
     id:=Node.FindNode('id');
     id_value:=id.NodeValue;
     x:=Node.FindNode('x');
     x_value:=x.NodeValue;
     y:=Node.FindNode('y');
     y_value:=y.NodeValue;
     def:=Node.FindNode('Defined');
     def_value:=def.NodeValue;
     nl:=Node.FindNode('Number_of_Links');
     nl_value:=nl.NodeValue;
     l1:=length(nodelist);
     setlength(nodelist,l1+1)
     nodelist[l1+1].pos_X:=floattoint(x_value);
     nodelist[l1+1].pos_y:=floattoint(y_value);
     nodelist[l1+1].defined:=strtoint(def_value);
     nodelist[l1+1].number_of_links:=strtoint(nl_value);
  end;
end;

end.

