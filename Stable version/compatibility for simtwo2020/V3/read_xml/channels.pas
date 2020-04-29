unit channels;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils;

type

 TProcessFrame = procedure(channel: char; value: integer; source: integer) of object;

{ TChannels }

TChannels = class
private
public
  serialData: string;
  FrameDigits: integer;

  channel: char;
  frame, frameSource: integer;
  frameData: string;

  ProcessFrame: TProcessFrame;
  procedure ReceiveData(s: string);


  constructor Create(newProcessFrame: TProcessFrame; NewFrameDigits: integer = 8);
  destructor Destroy; override;
end;


function isHexDigit(c: char): boolean;

implementation

function isHexDigit(c: char): boolean;
begin
  result := c in ['0'..'9', 'A'..'F'];
end;


{ TChannels }

constructor TChannels.Create(newProcessFrame: TProcessFrame; NewFrameDigits: integer);
begin
  ProcessFrame:= newProcessFrame;
  FrameDigits := NewFrameDigits;
end;

destructor TChannels.Destroy;
begin
  inherited Destroy;
end;


procedure TChannels.ReceiveData(s: string);
var //b: byte;
    c: char;
    value: integer;
begin
  if s = '' then exit;
  serialData := serialData + s;

  while Length(serialData) > 0 do begin
    c := serialData[1];
    serialData := copy(serialData, 2, maxint);
    if frame = -1 then begin

      if c = '*' then frameSource := 0
      else if c = '+' then frameSource := 1
      else if c = '-' then frameSource := 2;

      if (c in ['G'..'Z']) or (c in ['g'..'z']) then begin
        frame := 0;
        channel := c;
        frameData := '';
      end;
    end else begin
      if isHexDigit(c) then begin
        frameData := frameData + c;
        inc(frame);
        if frame = FrameDigits then begin
          value := StrToIntDef('$' + frameData, -1);
          processFrame(channel, value, frameSource);
          frame := -1;
        end;
      end else begin
        frame := -1;
      end;
    end;
  end;
end;

end.
