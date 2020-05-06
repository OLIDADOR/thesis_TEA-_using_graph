const
  NumJoints = 4;
  NumAxis = 4;

// Global Variables
var
  d1, d2, d3: double;
  PosJ: array[0..NumJoints - 1] of double;
  RefJ: array[0..NumJoints - 1] of double;
  ReqRefJ: array[0..NumJoints - 1] of double;
  JoyAxis: array[0..NumAxis - 1] of integer;
  JoyButtons: integer;
  seq, seqLen: integer;
  count: integer;

procedure Control;
var i, ri: integer;
    mess, s: string;
    err, speed: double;
begin
  inc(count);
  if count > 10 then begin
    //WriteUDPData('127.0.0.1', 9909, 'tick');
    count := 0;
  end;
  mess := ReadUDPData();
  if mess <> '' then begin
    SetRCValue(8, 1, inttostr(length(mess)));
    // If has the length of a valid message
    if length(mess) >= 2 + 5 * 8 then begin   // 2 byte header plus 5 (8 byte) chunks
      // Header
      s := copy(mess, 1, 2);
      if s = 'JY' then begin
        // Read buttons
        s := copy(mess, 2 + 1, 8);
        JoyButtons := StrToIntDef('$' + s, 0);
        // Read Joystick axis
        for i := 0 to 3 do begin
           s := copy(mess, 2 + (i + 1) * 8 + 1, 8); // Slice the message
           JoyAxis[i] := StrToIntDef('$' + s, 0);
           SetRCValue(4 + i, 6, format('%d',[JoyAxis[i]]));
        end;
        // Use them to control the Joints
        for i := 0 to 2 do begin
          RefJ[i] := RefJ[i] + rad(JoyAxis[i] / 32767.0); // TODO: put limits here
        end;
        // Use the buttons to control the gripper
        if (JoyButtons and (1 shl 2)) <> 0 then begin
          RefJ[3] := RefJ[3] + rad(2);
          if deg(RefJ[3]) > 90 then RefJ[3] := rad(90);
        end;
        if (JoyButtons and (1 shl 1)) <> 0 then begin
          RefJ[3] := RefJ[3] - rad(2);
          if RefJ[3] < 0 then RefJ[3] := 0;
        end;
      end;

      if s = 'SA' then begin
        // Read Servo angles
        for i := 0 to 3 do begin
           s := copy(mess, 2 + i * 8 + 1, 8); // Slice the message
           RefJ[i] := rad(StrToIntDef('$' + s, 0));
        end;
      end;

    end;
  end;
  
  // Movement Joints
  for i := 0 to 2 do begin
    PosJ[i] := GetAxisPos(0, i);
    ri := 4 + i;
    SetRCValue(ri, 2, format('%3.2f',[Deg(PosJ[i])]));

    if RCButtonPressed(ri, 4) then begin
      RefJ[i] := Rad(GetRCValue(ri, 5));
    end;

    SetRCValue(4 + i, 3,format('%3.2f',[deg(RefJ[i])]));
    SetAxisPosRef(0, i, RefJ[i]);
  end;

  // Gripper
  PosJ[3] := (GetAxisPos(0, 4) + GetAxisPos(0, 4)) / 2;
  SetRCValue(7, 2,format('%3.2f',[Deg(PosJ[3])]));

  if RCButtonPressed(7, 4) then begin
    RefJ[3] := rad(GetRCValue(7, 5));
  end;

  SetRCValue(7, 3,format('%3.2f',[deg(RefJ[i])]));
  SetAxisPosRef(0, 4, RefJ[3]);
  SetAxisPosRef(0, 5, -RefJ[3]);

  
  // Start sequence
  if RCButtonPressed(9, 1) then begin
    seqLen := 0;
    while GetRCText(10, 2 + seqLen) <> '' do begin
      inc(seqLen);
    end;
    if seqLen > 0 then seq := 0;
  end;

  if seq >= 0 then begin
    err := 0;
    speed := rad(GetRCValue(14, 2 + seq)) / 50.0;
    if speed = 0 then speed := rad(1) / 50.0;
    // Movement Joints
    for i := 0 to 3 do begin
      ri := 10 + i;
      //RefJ[i] := rad(GetRCValue(ri, 2 + seq));
      ReqRefJ[i] := rad(GetRCValue(ri, 2 + seq));
      RefJ[i] := RefJ[i] - Sat(RefJ[i] -  ReqRefJ[i], speed);

      if i <> 3 then begin
        SetAxisPosRef(0, i, RefJ[i]);
      end else begin
        SetAxisPosRef(0, 4, RefJ[3]);
        SetAxisPosRef(0, 5, -RefJ[3]);
      end;
      
      //err := err + abs(PosJ[i] - RefJ[i]);
      err := err + abs(PosJ[i] - ReqRefJ[i]);
    end;
    // Test distance to reference
    if deg(err) < 2 then begin
      inc(seq);
      if seq >= seqLen then begin
        seq := -1;
      end;
    end;
  end;
  
  SetRCValue(10, 1,format('%d %d',[seq, seqLen]));
  SetRCValue(11, 1,format('%3.2f',[deg(err)]));


end;

procedure Initialize;
begin
  d1 := 0.1;
  d2 := 0.4;
  d3 := 0.3;
  
  seq := -1;
  WriteUDPData('127.0.0.1', 9909, 'tick');
end;
