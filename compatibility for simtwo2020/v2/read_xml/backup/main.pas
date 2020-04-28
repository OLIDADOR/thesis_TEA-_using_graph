unit main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, lNetComponents, Forms, Controls, Graphics,
  Dialogs, StdCtrls, ExtCtrls, IniPropStorage, SdpoSerial, LCLIntf, LCLType,
  ComCtrls, channels, math, lNet;

type

  { TFMain }

  TFMain = class(TForm)
    BStop: TButton;
    BOpenSerial: TButton;
    BCloseSerial: TButton;
    BSendRaw: TButton;
    BConfigSet: TButton;
    BSerialSend: TButton;
    BUDPSend: TButton;
    BGo: TButton;
    Button1: TButton;
    Button2: TButton;
    CBComBlinkMute: TCheckBox;
    CBRawDebug: TCheckBox;
    CBAutoOpen: TCheckBox;
    EditM0SetAcc: TEdit;
    EditM1SetAcc: TEdit;
    EditM0Speed: TEdit;
    EditBatVoltageRaw: TEdit;
    EditBatVoltageConv: TEdit;
    EditM0SetSpeed: TEdit;
    EditM1Speed: TEdit;
    EditM1SetSpeed: TEdit;
    EditSendChannel: TEdit;
    EditSendValue: TEdit;
    EditSerialName: TEdit;
    EditSendRaw: TEdit;
    EditBatVoltageTresh: TEdit;
    EditUDPPort: TEdit;
    EditBatVoltage: TEdit;
    EditUDPIP: TEdit;
    EditUDPSendPort: TEdit;
    IniPropStorage: TIniPropStorage;
    Label1: TLabel;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    ShapeComBlink: TShape;
    ShapeUDPState: TShape;
    TabData: TTabSheet;
    UDP: TLUDPComponent;
    MemoDebug: TMemo;
    PageControl: TPageControl;
    Serial: TSdpoSerial;
    ShapeSerialState: TShape;
    StatusBar: TStatusBar;
    TabDebug: TTabSheet;
    TabConfig: TTabSheet;
    procedure BCloseSerialClick(Sender: TObject);
    procedure BConfigSetClick(Sender: TObject);
    procedure BGoClick(Sender: TObject);
    procedure BOpenSerialClick(Sender: TObject);
    procedure BSendRawClick(Sender: TObject);
    procedure BUDPSendClick(Sender: TObject);
    procedure Button1Click(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure EditSendRawKeyDown(Sender: TObject; var Key: Word;
      Shift: TShiftState);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure SerialRxData(Sender: TObject);
    procedure UDPReceive(aSocket: TLSocket);
  private
    procedure Debug(s: string);
    procedure processFrame(channel: char; value: integer; source: integer);
    procedure SendUDPChannel(c: char; val: integer);
    procedure ShowSerialState;
    procedure ShowUDPState;
    { private declarations }
  public
    BatVoltageTresh, BatVoltageConv: double;
    SerialChannels, UDPChannels: TChannels;
    BatVoltage: double;
    BatVoltageRaw: integer;


    procedure SendMessage(c: char; val: integer);
  end;

//procedure ControlLoop(Form:TFMain;var agvs:RobotsTeam;var CaminhosAgvs:Caminhos;xCam:float;yCam:float;thetaCam:float);

var
  FMain: TFMain;


implementation

{$R *.lfm}

{ TFMain }


procedure ControlLoop(Form:TFMain;var agvs:RobotsTeam;var CaminhosAgvs:Caminhos;xCam:float;yCam:float;thetaCam:float);
//var M0Speed, M1Speed: integer;
//    M0Acc, M1Acc: integer;
//    msg, addr: string;
//    flagFollowLineOK, flagFollowCircleOK: boolean;
//    threshold: float;
//    theta_ref: integer;
//    v,w,v_nominal: float;
//    x_ref,y_ref: float;
//    Kd,Ktheta,dist,angle: float;
//    wf: float;
//    raioCurvatura: float;
//begin
//
//   //Inicializar flags e constantes
//   flagFollowLineOK := false;
//   flagFollowCircleOK := false;
//   threshold := 0.05;
//   raioCurvatura := 0.2291; //sqrt(0.162^2+0.162^2) -> 0.162 largura entre paredes
//   wf := v_nominal/raioCurvatura;
//
//   //Inicializar posições de todos os robôs
//   //(Tem de ser fora do Loop)
//    //agvs[0].InitialPoint.x:=2;
//    //agvs[0].InitialPoint.y:=4;
//    //agvs[0].InitialPoint.direction:=2;
//
//    //TEA* é executado a uma cadência fixa com base nas últimas posições lidas
//    //pela câmara.
//    //Assume como posição inicial do planeamento a célula aproximada consoante
//    //as medidas da câmara.
//
//    //Verifica se posição lida pela câmara já corresponde com o target point
//    //(se sim nada é feito, apenas permanece imóvel)
//
//
//
//    if ((xCam = agvs[0].TargetPoint.x) and (yCam = agvs[0].TargetPoint.y)) then begin
//      v := 0;
//      w := 0;
//    end;
//
//    //Verifica se posição (x,y) lida pela câmara corresponde já com o ponto
//    //central da célula definida como inicial a menos de um threshold, ou
//    //se já se encontra entre a célula 0 e 1
//
//    case CaminhosAgvs[0].coords[1].direction of
//         0: begin
//              if ((CaminhosAgvs[0].coords[0].y - yCam <= threshold) or
//                  ((yCam <= CaminhosAgvs[0].coords[1].y) and (yCam >= CaminhosAgvs[0].coords[0].y)))
//              then begin
//                     flagFollowLineOK:=true;
//                     theta_ref:=90;
//                     x_ref:=(CaminhosAgvs[0].coords[0].x-1)*0.09;
//                     dist:=xCam-x_ref;
//              end;
//            end;
//         1: begin
//              if (((CaminhosAgvs[0].coords[0].x - xCam <= threshold) and (CaminhosAgvs[0].coords[0].y - yCam <= threshold)) or
//                  ((xCam <= CaminhosAgvs[0].coords[1].x) and (xCam >= CaminhosAgvs[0].coords[0].x) and
//                   (yCam <= CaminhosAgvs[0].coords[1].y) and (yCam >= CaminhosAgvs[0].coords[0].y)))
//              then begin
//                     flagFollowCircleOK:=true;
//                     theta_ref:=45;
//                     //followCircle
//              end;
//            end;
//         2: begin
//              if ((CaminhosAgvs[0].coords[0].x - xCam <= threshold) or
//                  ((xCam <= CaminhosAgvs[0].coords[1].x) and (xCam >= CaminhosAgvs[0].coords[0].x)))
//              then begin
//                     flagFollowLineOK:=true;
//                     theta_ref:=0;
//                     y_ref:=(CaminhosAgvs[0].coords[0].y-1)*0.09;
//                     dist:=yCam-y_ref;
//              end;
//            end;
//         3: begin
//              if (((CaminhosAgvs[0].coords[0].x - xCam <= threshold) and (yCam - CaminhosAgvs[0].coords[0].y <= threshold)) or
//                  ((xCam <= CaminhosAgvs[0].coords[1].x) and (xCam >= CaminhosAgvs[0].coords[0].x) and
//                   (yCam >= CaminhosAgvs[0].coords[1].y) and (yCam <= CaminhosAgvs[0].coords[0].y)))
//              then begin
//                     flagFollowCircleOK:=true;
//                     theta_ref:=315;
//                     //followCircle
//              end;
//            end;
//         4: begin
//              if ((yCam - CaminhosAgvs[0].coords[0].y <= threshold) or
//                  ((yCam >= CaminhosAgvs[0].coords[1].y) and (yCam <= CaminhosAgvs[0].coords[0].y)))
//              then begin
//                     flagFollowLineOK:=true;
//                     theta_ref:=270;
//                     x_ref:=(CaminhosAgvs[0].coords[0].x-1)*0.09;
//                     dist:=xCam-x_ref;
//              end;
//            end;
//         5: begin
//              if (((xCam - CaminhosAgvs[0].coords[0].x <= threshold) and (yCam - CaminhosAgvs[0].coords[0].y <= threshold)) or
//                  ((xCam >= CaminhosAgvs[0].coords[1].x) and (xCam <= CaminhosAgvs[0].coords[0].x) and
//                   (yCam >= CaminhosAgvs[0].coords[1].y) and (yCam <= CaminhosAgvs[0].coords[0].y)))
//              then begin
//                     flagFollowCircleOK:=true;
//                     theta_ref:=225;
//                     //followCircle
//              end;
//            end;
//         6: begin
//              if ((xCam - CaminhosAgvs[0].coords[0].x <= threshold) or
//                  ((xCam >= CaminhosAgvs[0].coords[1].x) and (xCam <= CaminhosAgvs[0].coords[0].x)))
//              then begin
//                     flagFollowLineOK:=true;
//                     theta_ref:=180;
//                     y_ref:=(CaminhosAgvs[0].coords[0].y-1)*0.09;
//                     dist:=yCam-y_ref;
//              end;
//            end;
//         7: begin
//              if (((xCam - CaminhosAgvs[0].coords[0].x <= threshold) and (CaminhosAgvs[0].coords[0].y - yCam <= threshold)) or
//                  ((xCam >= CaminhosAgvs[0].coords[1].x) and (xCam <= CaminhosAgvs[0].coords[0].x) and
//                   (yCam <= CaminhosAgvs[0].coords[1].y) and (yCam >= CaminhosAgvs[0].coords[0].y)))
//              then begin
//                     flagFollowCircleOK:=true;
//                     theta_ref:=135;
//                     //followCircle
//              end;
//            end;
//    end;
//
//    // Define nova reta entre o ponto inicial e o ponto seguinte
//    //(em coordenadas (x,y))
//    if flagFollowLineOK = true then begin
//        angle:=thetaCam-theta_ref;
//        v:=v_nominal;
//        w:=Kd*dist+Ktheta*angle;
//    end
//    else if flagFollowCircleOK = true then begin
//        angle:=thetaCam-theta_ref;
//        v:=v_nominal;
//        w:=Kd*dist+Ktheta*angle+wf;
//    end;
//    //else begin
//        // Define reta entre o ponto atual e o ponto seguinte
//        //(em coordenadas (x,y))
//    //end;
//
//    //Enquanto não houver leitura da câmara mantém-se as velocidades dadas e
//    //segue-se a linha já anteriormente definida
//
//
//end;


procedure TFMain.ShowUDPState;
begin
  if UDP.Connected then begin
    ShapeUDPState.Brush.Color := clGreen;
  end else begin
    ShapeUDPState.Brush.Color := clRed;
  end;
end;


procedure TFMain.ShowSerialState;
begin
  if Serial.Active then begin
    ShapeSerialState.Brush.Color := clGreen;
  end else begin
    ShapeSerialState.Brush.Color := clRed;
  end;
end;


procedure TFMain.SendMessage(c: char; val: integer);
begin
  Serial.WriteData(c + IntToHex(dword(Val), 8));
end;


procedure TFMain.SendUDPChannel(c: char; val: integer);
var addr: string;
begin
  addr := EditUDPIP.Text + ':' + EditUDPSendPort.Text;
  UDP.SendMessage(c + IntToHex(dword(Val), 8), addr);
end;


procedure TFMain.BOpenSerialClick(Sender: TObject);
begin
  Serial.Device := EditSerialName.Text;
  Serial.Open;
  ShowSerialState();
end;


procedure TFMain.BSendRawClick(Sender: TObject);
var addr: string;
begin
  if Serial.Active then Serial.WriteData(EditSendRaw.Text);
  if UDP.Active then begin
    addr := EditUDPIP.Text + ':' + EditUDPSendPort.Text;
    UDP.SendMessage(EditSendRaw.Text, addr);
  end;

end;


procedure TFMain.BUDPSendClick(Sender: TObject);
var s: string;
  v: integer;
begin
  s := EditSendChannel.Text;
  if s <> '' then begin
    v := StrToIntDef(EditSendValue.Text, 0);
    SendUDPChannel(s[1], v);
  end;
end;


procedure TFMain.Button1Click(Sender: TObject);
var M0Speed, M1Speed: integer;
    M0Acc, M1Acc: integer;
    msg, addr: string;
begin
  //M0Acc := 100;
  //M1Acc := 100;
  //M0Speed := -1500;
  //M1Speed := 1500;
  //addr := EditUDPIP.Text + ':' + EditUDPSendPort.Text;
  //msg := 'S' + '00' + IntToHex(byte(M0Acc), 2) + IntToHex(word(M0Speed), 4);
  //msg += 'S' + '01' + IntToHex(byte(M1Acc), 2) + IntToHex(word(M1Speed), 4);
  //UDP.SendMessage(msg, addr);
end;


procedure TFMain.Button2Click(Sender: TObject);
var M0Speed, M1Speed: integer;
    M0Acc, M1Acc: integer;
    msg, addr: string;
begin
  M0Speed := 0;
  M1Speed := 0;
  M0Acc := 255;
  M1Acc := 255;
  addr := EditUDPIP.Text + ':' + EditUDPSendPort.Text;
  msg := 'S' + '00' + IntToHex(byte(M0Acc), 2) + IntToHex(word(M0Speed), 4);
  msg += 'S' + '01' + IntToHex(byte(M1Acc), 2) + IntToHex(word(M1Speed), 4);
  UDP.SendMessage(msg, addr);
end;


procedure TFMain.EditSendRawKeyDown(Sender: TObject; var Key: Word;
  Shift: TShiftState);
begin
  if key = VK_RETURN then begin
    BSendRaw.Click();
  end;
end;


procedure TFMain.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  Serial.Close;
end;


procedure TFMain.FormCreate(Sender: TObject);
begin
  DefaultFormatSettings.DecimalSeparator := '.';
  SerialChannels := TChannels.Create(@processFrame);
  UDPChannels := TChannels.Create(@processFrame);
end;


procedure TFMain.FormShow(Sender: TObject);
var port: integer;
begin
  ShowSerialState();
  BConfigSet.Click();
  if CBAutoOpen.Checked then BOpenSerial.Click();

  port := StrToIntDef(EditUDPPort.Text, 9632);
  UDP.Listen(port);
  EditUDPPort.Text := IntToStr(port);
  ShowUDPState();
end;


procedure TFMain.FormDestroy(Sender: TObject);
begin
  UDPChannels.Free;
  SerialChannels.Free;
end;


procedure TFMain.Debug(s: string);
begin
  //MemoDebug.VertScrollBar.Position := MemoDebug.VertScrollBar.Range;
  MemoDebug.Text := MemoDebug.Text + s;
  MemoDebug.SelStart:=Length(MemoDebug.Text);
  while MemoDebug.Lines.Count > 1000 do begin
    MemoDebug.Lines.Delete(0);
  end;
end;


procedure TFMain.SerialRxData(Sender: TObject);
var s: string;
begin
  s := Serial.ReadData;
  if s = '' then exit;

  SerialChannels.ReceiveData(s);

  if CBRawDebug.Checked then begin
    Debug(s);
  end;
end;


procedure TFMain.UDPReceive(aSocket: TLSocket);
var msg: string;
begin
  UDP.GetMessage(msg);
  if msg = '' then exit;

  UDPChannels.ReceiveData(msg);

  if CBRawDebug.Checked then begin
    Debug(msg);
  end;

end;


procedure TFMain.BCloseSerialClick(Sender: TObject);
begin
  Serial.Close;
  ShowSerialState();
end;


procedure TFMain.BConfigSetClick(Sender: TObject);
begin
  BatVoltageConv := StrToFloat(EditBatVoltageConv.Text);
  BatVoltageTresh := StrToFloat(EditBatVoltageTresh.Text);
end;


procedure TFMain.BGoClick(Sender: TObject);
var M0Speed, M1Speed: integer;
    M0Acc, M1Acc: integer;
    msg, addr: string;
begin
  if (sender as TButton).Cancel then begin
    M0Speed := 0;
    M1Speed := 0;

    M0Acc := 255;
    M1Acc := 255;
  end else begin
    M0Speed := StrToIntDef(EditM0SetSpeed.Text, 0);
    M1Speed := StrToIntDef(EditM1SetSpeed.Text, 0);

    M0Acc := StrToIntDef(EditM0SetAcc.Text, 100);
    M1Acc := StrToIntDef(EditM1SetAcc.Text, 100);
  end;

  addr := EditUDPIP.Text + ':' + EditUDPSendPort.Text;
  msg := 'S' + '00' + IntToHex(byte(M0Acc), 2) + IntToHex(word(M0Speed), 4);
  msg += 'S' + '01' + IntToHex(byte(M1Acc), 2) + IntToHex(word(M1Speed), 4);
  UDP.SendMessage(msg, addr);
end;


procedure TFMain.processFrame(channel: char; value: integer; source: integer);
var i: integer;
    s: string;
begin
  //MemoDebug.Text := MemoDebug.Text + channel;
  if channel = 'i' then begin
  //end else if channel = 'r' then begin
    // if the arduino was reset ...
  end else if channel = 'V' then begin
    BatVoltageRaw := value;
    BatVoltage := value * BatVoltageConv;
    EditBatVoltageRaw.Text := format('%d', [BatVoltageRaw]);
    EditBatVoltage.Text := format('%.2f', [BatVoltage]);


    if not CBComBlinkMute.Checked then begin
      if ShapeComBlink.Brush.Color <> clGreen then begin
        ShapeComBlink.Brush.Color := clGreen;
      end else begin
        ShapeComBlink.Brush.Color := clLime;
      end;
    end;
  end else if channel = 'M' then begin
    EditM0Speed.Text := format('%d', [value]);
  end else if channel = 'N' then begin
    EditM1Speed.Text := format('%d', [value]);

  end else if channel in ['s', 't'] then begin
    i := 1 + ord(channel) - ord('r');

  end;
end;


end.
